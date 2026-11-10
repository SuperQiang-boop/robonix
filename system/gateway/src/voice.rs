// SPDX-License-Identifier: MulanPSL-2.0
// voice.rs — voice push-to-talk orchestrator behind SystemGateway.StartVoiceSession.
//
// One call drives the full pipeline:
//
//   RobonixPrimitiveAudioMic.Stream  (record_seconds)
//       │
//       ▼
//   SystemSpeechAsr.Recognize    → transcript
//       │
//       ▼
//   RobonixServiceVoiceprintIdentify.Identify → user_id  (graceful: hint fallback on absence)
//       │
//       ▼
//   Build planner::Task { source=AUDIO, text=transcript, user_id, … }
//       │
//       ▼
//   RobonixSystemPlanner.SubmitTask      → forward each PlannerEvent as VoiceEvent { kind=PLANNER, … }
//       │
//       ▼
//   (if tts_enabled) speech/speak, or speech/tts → audio/speaker when absent
//       │
//       ▼
//   VoiceEvent { kind=SESSION_DONE }
//
// Mock mode (`ROBONIX_GATEWAY_VOICE_MOCK=1` or `ROBONIX_GATEWAY_VOICE_MOCK_TEXT`):
// skip mic+ASR, use a canned transcript so the link can be exercised without
// audio hardware. This is what the demo script uses by default.

use anyhow::Result;
use futures::Stream;
use rmcp::ServiceExt;
use robonix_src::client::SrcClient;
use robonix_src::pb as src_pb;
use std::fs;
use std::path::{Path, PathBuf};
use std::pin::Pin;
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
use std::task::{Context, Poll};
use std::time::Duration;
use tokio::sync::{Mutex, mpsc};
use tokio_stream::{StreamExt, wrappers::ReceiverStream};
use tonic::{Request, Status};
use uuid::Uuid;

use crate::pb::audio::AudioChunk;
use crate::pb::contracts::{
    robonix_primitive_audio_mic_client::RobonixPrimitiveAudioMicClient,
    robonix_primitive_audio_speaker_client::RobonixPrimitiveAudioSpeakerClient,
    robonix_service_speech_asr_stream_client::RobonixServiceSpeechAsrStreamClient,
    robonix_service_speech_tts_client::RobonixServiceSpeechTtsClient,
    robonix_service_voiceprint_identify_client::RobonixServiceVoiceprintIdentifyClient,
    robonix_system_planner_client::RobonixSystemPlannerClient,
};
use crate::pb::gateway::{StartVoiceSessionRequest, VoiceEvent};
use crate::pb::planner::{PlannerEvent, Task};
use crate::pb::tts;
use crate::pb::voiceprint;
use crate::{access, access::AccessControlConfig};
use robonix_log::{info, warn};

// ── Stable VoiceEvent kinds (mirror lib/system/gateway/msg/VoiceEvent.msg) ───

pub const KIND_SESSION_STARTED: u32 = 0;
pub const KIND_RECORDING_STARTED: u32 = 1;
pub const KIND_RECORDING_DONE: u32 = 2;
#[allow(dead_code)]
pub const KIND_ASR_PARTIAL: u32 = 3;
pub const KIND_ASR_FINAL: u32 = 4;
pub const KIND_USER_IDENTIFIED: u32 = 5;
pub const KIND_PLANNER: u32 = 6;
pub const KIND_TTS_STARTED: u32 = 7;
pub const KIND_TTS_DONE: u32 = 8;
pub const KIND_SESSION_DONE: u32 = 9;
pub const KIND_ERROR: u32 = 10;

/// Hard ceiling on a single voice turn. Real end-of-turn comes from
/// the silence-VAD in the mic pump (see VAD_* below); this just keeps
/// a stuck mic from monopolizing the session forever.
const DEFAULT_RECORD_SECONDS: u32 = 10;

/// Silence-VAD parameters. After we've heard a chunk above
/// `VAD_SPEECH_RMS`, count consecutive sub-threshold chunks; once the
/// silence lasts `VAD_END_SILENCE_SECS`, treat the utterance as done
/// and close the ASR request stream so the server flushes its final.
/// Tuned for 16 kHz mono s16le speech: speech RMS is typically 1k–8k,
/// background noise sits around 50–300, so 500 is a comfortable gap.
const VAD_SPEECH_RMS: f32 = 500.0;
const VAD_END_SILENCE_SECS: f32 = 1.2;
const VAD_NO_SPEECH_TIMEOUT_SECS: u64 = 5;
const DEFAULT_ASR_LANGUAGE: &str = "";
const DEFAULT_AUDIO_ENCODING: &str = "pcm_s16le";
const DEFAULT_AUDIO_SAMPLE_RATE: u32 = 16_000;

struct AbortOnDrop(tokio::task::AbortHandle);

impl Drop for AbortOnDrop {
    fn drop(&mut self) {
        self.0.abort();
    }
}

struct VoiceSessionStream {
    inner: ReceiverStream<Result<VoiceEvent, Status>>,
    task: tokio::task::JoinHandle<()>,
}

impl Stream for VoiceSessionStream {
    type Item = Result<VoiceEvent, Status>;

    fn poll_next(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        Pin::new(&mut self.inner).poll_next(cx)
    }
}

impl Drop for VoiceSessionStream {
    fn drop(&mut self) {
        self.task.abort();
    }
}

// ── Discovery helpers ────────────────────────────────────────────────────────
//
// Src → contract_id → endpoint resolution. Every speech / mic / speaker
// service registers its gRPC capability with `metadata_json = {"endpoint":
// "host:port"}` (see audio_driver/node.py and speech_service/service.py).

/// Two-step endpoint resolution: query for providers offering `contract_id`, then
/// `ConnectCapability` against the chosen one to actually receive the
/// endpoint string. Src hides endpoints from `query_capabilities` on
/// purpose; consumers must commit a channel before they can dial.
pub(crate) async fn resolve_endpoint(
    src: &Arc<Mutex<SrcClient>>,
    contract_id: &str,
    pin_provider_id: &str,
) -> Option<String> {
    let mut src = src.lock().await;
    let transport = src_pb::Transport::Grpc;
    let providers = src
        .query_capabilities("", contract_id, transport)
        .await
        .ok()?;

    // Auto-pick path: any provider providing this contract over the right transport.
    let auto_pick = || -> Option<&src_pb::CapabilityProvider> {
        providers.iter().find(|r| {
            r.capabilities
                .iter()
                .any(|i| i.contract_id == contract_id && i.transport == transport as i32)
        })
    };

    let pick: Option<&src_pb::CapabilityProvider> = if pin_provider_id.is_empty() {
        auto_pick()
    } else {
        // A caller that explicitly names a provider is selecting a physical I/O
        // boundary, not merely expressing a preference. Falling back from a
        // client bridge to the robot device would silently capture or play on
        // the wrong machine, so an explicit pin is a hard requirement.
        match providers
            .iter()
            .find(|r| r.id == pin_provider_id || r.namespace == pin_provider_id)
        {
            Some(provider) => Some(provider),
            None => {
                warn!(
                    "[voice] pinned provider '{pin_provider_id}' for {contract_id} not in src; \
                     refusing implicit fallback. Available providers: {:?}",
                    providers.iter().map(|r| r.id.as_str()).collect::<Vec<_>>()
                );
                None
            }
        }
    };

    let provider = pick?;
    let (_channel_id, endpoint, _params) = src
        .connect_capability("gateway", &provider.id, contract_id, transport)
        .await
        .ok()?;
    if endpoint.is_empty() {
        return None;
    }
    Some(localhost_to_ipv4(&endpoint))
}

fn localhost_to_ipv4(ep: &str) -> String {
    let with_scheme = if ep.starts_with("http") {
        ep.to_string()
    } else {
        format!("http://{ep}")
    };
    with_scheme.replace("localhost", "127.0.0.1")
}

// ── Public entry point ───────────────────────────────────────────────────────

/// Run one voice session. Returns the receiver immediately; orchestration
/// runs on a spawned task and pushes `VoiceEvent`s as it progresses.
///
/// `planner_endpoint_default` is the http URL Gateway was configured with at
/// boot — used when Src does not yet know about Planner.
pub async fn start_voice_session(
    req: StartVoiceSessionRequest,
    src: Arc<Mutex<SrcClient>>,
    planner_endpoint_default: String,
    access: Arc<AccessControlConfig>,
) -> Result<impl Stream<Item = Result<VoiceEvent, Status>>, Status> {
    let session_id = if req.session_id.is_empty() {
        Uuid::new_v4().to_string()
    } else {
        req.session_id.clone()
    };
    let record_seconds = if req.record_seconds == 0 {
        DEFAULT_RECORD_SECONDS
    } else {
        req.record_seconds
    };
    let language = if req.language.is_empty() {
        DEFAULT_ASR_LANGUAGE.to_string()
    } else {
        req.language.clone()
    };

    let (tx, rx) = mpsc::channel::<Result<VoiceEvent, Status>>(64);

    let _ = tx
        .send(Ok(event_status(
            KIND_SESSION_STARTED,
            &session_id,
            &format!(
                "voice session started (vad, record≤{record_seconds}s, no-speech≤{VAD_NO_SPEECH_TIMEOUT_SECS}s, tts={}, lang={})",
                req.tts_enabled,
                if language.is_empty() {
                    "auto"
                } else {
                    &language
                },
            ),
        )))
        .await;

    let session_id_for_task = session_id.clone();
    let task = tokio::spawn(async move {
        let outcome = run_session(
            req,
            session_id_for_task.clone(),
            record_seconds,
            language,
            src,
            planner_endpoint_default,
            access,
            tx.clone(),
        )
        .await;

        match outcome {
            Ok(()) => {
                let _ = tx
                    .send(Ok(event_status(
                        KIND_SESSION_DONE,
                        &session_id_for_task,
                        "voice session done",
                    )))
                    .await;
            }
            Err(e) => {
                let _ = tx
                    .send(Ok(event_error(&session_id_for_task, &format!("{e:#}"))))
                    .await;
            }
        }
    });

    Ok(VoiceSessionStream {
        inner: ReceiverStream::new(rx),
        task,
    })
}

#[allow(clippy::too_many_arguments)]
async fn run_session(
    req: StartVoiceSessionRequest,
    session_id: String,
    record_seconds: u32,
    language: String,
    src: Arc<Mutex<SrcClient>>,
    planner_endpoint_default: String,
    access: Arc<AccessControlConfig>,
    tx: mpsc::Sender<Result<VoiceEvent, Status>>,
) -> Result<()> {
    let mock = is_mock_mode();
    // `record_seconds` is now a hard-stop ceiling on streaming capture
    // (FunASR's VAD ends the turn under most conditions; this just
    // protects against a sensor that never goes silent). 0 / unset →
    // 30 s default. Whisper's old "record-then-recognize" pattern is
    // gone; voice goes through `asr_stream` end-to-end.
    let max_seconds: u32 = if record_seconds == 0 {
        30
    } else {
        record_seconds.max(5)
    };

    let t_cap = std::time::Instant::now();
    let (audio_pcm, transcript) = if mock {
        let canned = mock_transcript();
        let _ = tx
            .send(Ok(event_status(
                KIND_RECORDING_STARTED,
                &session_id,
                "mock mode — skipping mic capture",
            )))
            .await;
        let _ = tx
            .send(Ok(event_status(
                KIND_RECORDING_DONE,
                &session_id,
                &format!("mock transcript ready ({} bytes)", canned.len()),
            )))
            .await;
        let _ = tx
            .send(Ok(event_text(KIND_ASR_FINAL, &session_id, &canned, 1.0)))
            .await;
        (Vec::new(), canned)
    } else {
        // Mic → AsrAudioChunk pump + asr_stream.RecognizeStream both
        // run inside `stream_capture_and_recognize`. It returns once
        // FunASR emits is_final, the user hits max_seconds, or the
        // mic stream drops. Voiceprint still gets the accumulated PCM
        // for downstream identification (currently fallback-only).
        stream_capture_and_recognize(
            &src,
            &req.mic_node_id,
            &req.asr_node_id,
            &language,
            max_seconds,
            &session_id,
            &tx,
            context_flag(&req.context_json, "barge_in"),
        )
        .await?
    };

    // [profile] record start → final transcript: covers mic-open + audio
    // transfer + streaming ASR inference. Compare against the per-chunk mic
    // timings above (transfer) and the speech service's [profile-asr] lines
    // (inference) to see which dominates.
    {
        let secs = audio_pcm.len() as f64 / (DEFAULT_AUDIO_SAMPLE_RATE as f64 * 2.0);
        info!(
            "[profile] record→transcript {} ms (audio {:.2}s, {} pcm bytes, transcript {} chars)",
            t_cap.elapsed().as_millis(),
            secs,
            audio_pcm.len(),
            transcript.chars().count()
        );
    }

    if transcript.trim().is_empty() {
        anyhow::bail!("empty transcript — nothing to send to Planner");
    }

    // 3. Voiceprint + access gate. ASR may already have produced a transcript,
    // but no Planner task, TTS, or action is allowed until the voice identity passes
    // the Gateway access policy. Client hints cannot bypass voiceprint when the
    // gate is enabled.
    let identity = identify_user(
        &src,
        &req.voiceprint_node_id,
        &audio_pcm,
        &req.client_user_id,
        &session_id,
        &tx,
    )
    .await;
    let decision = access.authorize_voice(&req.client_user_id, identity.response.as_ref());
    let (user_id, access_context) = match decision {
        access::AccessDecision::Allow {
            user_id,
            method,
            confidence,
            reason,
        } => {
            info!(
                "[gateway/access] voice allow user={user_id} via {method:?} confidence={confidence:.2}: {reason}"
            );
            let _ = tx
                .send(Ok(event_user(
                    KIND_USER_IDENTIFIED,
                    &session_id,
                    &user_id,
                    confidence,
                    &format!("access allowed: {reason}"),
                )))
                .await;
            (
                user_id,
                AccessContext {
                    method: method.as_str().to_string(),
                    confidence,
                    reason,
                },
            )
        }
        access::AccessDecision::Deny {
            user_id,
            confidence,
            reason,
        } => {
            warn!(
                "[gateway/access] voice deny user={user_id} confidence={confidence:.2}: {reason}"
            );
            anyhow::bail!("access denied for voice user '{user_id}': {reason}");
        }
    };

    // A disconnected capture must not submit a late command after greeting Stop.
    if tx.is_closed() {
        anyhow::bail!("voice session closed before task admission");
    }

    // 4. Build Task and stream Planner events.
    let planner_endpoint = resolve_endpoint(&src, "robonix/system/planner", "")
        .await
        .unwrap_or(planner_endpoint_default);

    let task = build_task(
        &session_id,
        &transcript,
        &user_id,
        &access_context,
        &audio_pcm,
        &req.context_json,
    );

    let mut accumulated_text = String::new();
    // Track whether we've already spoken at least one per-plan answer, so the
    // end-of-turn fallback below doesn't double-speak.
    let mut spoke_any = false;
    // [profile] planner round-trip (submit → first event → turn done).
    let t_planner = std::time::Instant::now();
    let mut first_planner_logged = false;
    // Per-narration-segment TTS buffer: text chunks accumulate here and flush
    // to speech at each plan/batch/final boundary (see the stream loop below).
    let mut seg = String::new();

    let planner_stream_result = async {
        let mut client = RobonixSystemPlannerClient::connect(planner_endpoint.clone())
            .await
            .map_err(|e| tonic::Status::unavailable(e.to_string()))?;
        let resp = client.submit_task(Request::new(task)).await?;
        Ok::<_, tonic::Status>(resp.into_inner())
    }
    .await;

    match planner_stream_result {
        Ok(mut planner_stream) => {
            while let Some(item) = planner_stream.next().await {
                match item {
                    Ok(ev) => {
                        if !first_planner_logged {
                            info!(
                                "[profile] planner: first event +{} ms (after transcript submit)",
                                t_planner.elapsed().as_millis()
                            );
                            first_planner_logged = true;
                        }
                        accumulate_text(&ev, &mut accumulated_text);
                        // Speak each narration segment the moment it completes.
                        // The model's per-plan answer streams as EVT_TEXT_CHUNK (0);
                        // a PLAN (1) / BATCH_RESULT (2) / FINAL_TEXT (4) event marks
                        // the end of that narration, so flush the buffered chunks to
                        // TTS. This voices every plan's full answer live, instead of
                        // only the tiny end-of-turn final_text.
                        let kind = ev.event_kind;
                        if kind == 0 {
                            seg.push_str(&ev.text_chunk);
                        }
                        // Status (3) is also a narration boundary. Planner uses
                        // it after an in-progress no-call round so TTS can play
                        // the complete progress update without mislabelling it
                        // as FinalText and closing the interaction stream.
                        let is_boundary = is_narration_boundary(kind);
                        let say = if is_boundary {
                            let streamed = std::mem::take(&mut seg);
                            if req.tts_enabled {
                                tts_boundary_text(kind, streamed, &ev.final_text)
                            } else {
                                None
                            }
                        } else {
                            None
                        };
                        let _ = tx
                            .send(Ok(VoiceEvent {
                                event_kind: KIND_PLANNER,
                                session_id: session_id.clone(),
                                text: String::new(),
                                user_id: user_id.clone(),
                                confidence: 0.0,
                                planner: Some(ev),
                                error: String::new(),
                                status_message: String::new(),
                                timestamp_ms: now_ms(),
                            }))
                            .await;
                        if let Some(s) = say {
                            spoke_any = true;
                            let t_tts = std::time::Instant::now();
                            let say_chars = s.chars().count();
                            if let Err(e) = synthesize_and_play(
                                &src,
                                &req.tts_node_id,
                                &req.speaker_node_id,
                                &language,
                                &s,
                                &session_id,
                                &tx,
                            )
                            .await
                            {
                                let _ = tx
                                    .send(Ok(event_status(
                                        KIND_TTS_DONE,
                                        &session_id,
                                        &format!("tts skipped: {e:#}"),
                                    )))
                                    .await;
                            }
                            info!(
                                "[profile] tts+play +{} ms ({} chars spoken)",
                                t_tts.elapsed().as_millis(),
                                say_chars
                            );
                        }
                    }
                    Err(e) => {
                        anyhow::bail!("Planner stream error: {e}");
                    }
                }
            }
        }
        Err(e) => {
            // No Planner online — surface as session ERROR rather than fabricating
            // a "message received successfully" reply (callers must see the failure).
            anyhow::bail!("Planner unreachable at {planner_endpoint}: {e}");
        }
    }

    info!(
        "[profile] planner: turn done +{} ms ({} chars of answer text)",
        t_planner.elapsed().as_millis(),
        accumulated_text.chars().count()
    );

    // 5. Flush any trailing narration the loop buffered but never hit a
    // boundary for (turn ended on chunks). Non-fatal on any error.
    let tail = if !seg.trim().is_empty() {
        std::mem::take(&mut seg)
    } else if !spoke_any {
        accumulated_text.clone()
    } else {
        String::new()
    };
    if req.tts_enabled
        && !tail.trim().is_empty()
        && let Err(e) = synthesize_and_play(
            &src,
            &req.tts_node_id,
            &req.speaker_node_id,
            &language,
            &tail,
            &session_id,
            &tx,
        )
        .await
    {
        let _ = tx
            .send(Ok(event_status(
                KIND_TTS_DONE,
                &session_id,
                &format!("tts skipped: {e:#}"),
            )))
            .await;
    }

    Ok(())
}

// ── Mic capture + streaming ASR ─────────────────────────────────────────────
//
// Open the mic primitive AND `robonix/service/speech/asr_stream` (FunASR
// paraformer-zh-streaming bidi) at the same time. Mic AudioChunks get
// wrapped as AsrAudioChunk and pumped up the request side; the response
// side yields RecognizeStreamEvents (PARTIAL → … → FINAL). On FINAL,
// stop both streams and return `(accumulated_pcm, final_text)`.
//
// `max_seconds` is a hard ceiling — FunASR's VAD usually ends the turn
// well before that, but we don't want to hang on a stuck mic.

#[allow(clippy::too_many_arguments)]
async fn stream_capture_and_recognize(
    src: &Arc<Mutex<SrcClient>>,
    mic_pin: &str,
    asr_pin: &str,
    language: &str,
    max_seconds: u32,
    session_id: &str,
    tx: &mpsc::Sender<Result<VoiceEvent, Status>>,
    barge_in: bool,
) -> Result<(Vec<u8>, String)> {
    let mic_endpoint = resolve_endpoint(src, "robonix/primitive/audio/mic", mic_pin)
        .await
        .ok_or_else(|| anyhow::anyhow!("no RobonixPrimitiveAudioMic provider registered in Src"))?;
    let asr_endpoint = resolve_endpoint(src, "robonix/service/speech/asr_stream", asr_pin)
        .await
        .ok_or_else(|| {
            anyhow::anyhow!("no RobonixServiceSpeechAsrStream provider registered in Src")
        })?;

    let mut mic_client = RobonixPrimitiveAudioMicClient::connect(mic_endpoint.clone())
        .await
        .map_err(|e| anyhow::anyhow!("connect mic at {mic_endpoint}: {e}"))?;
    let mut asr_client = RobonixServiceSpeechAsrStreamClient::connect(asr_endpoint.clone())
        .await
        .map_err(|e| anyhow::anyhow!("connect asr_stream at {asr_endpoint}: {e}"))?;

    let _ = tx
        .send(Ok(event_status(
            KIND_RECORDING_STARTED,
            session_id,
            &format!("streaming mic {mic_endpoint} → asr_stream {asr_endpoint}"),
        )))
        .await;

    let mut mic_request = Request::new(());
    if barge_in {
        mic_request.metadata_mut().insert(
            "x-robonix-barge-in",
            "true".parse().expect("static metadata value"),
        );
    }
    let mut mic_stream = mic_client
        .mic(mic_request)
        .await
        .map_err(|e| anyhow::anyhow!("mic rpc failed: {e}"))?
        .into_inner();

    // Buffered hand-off: mic-pump task drops AsrAudioChunks onto the
    // mpsc; tonic forwards them up the bidi as the request stream.
    let (asr_req_tx, asr_req_rx) = mpsc::channel::<crate::pb::asr::AsrAudioChunk>(64);
    let asr_req_stream = ReceiverStream::new(asr_req_rx);

    // Accumulator the mic-pump writes into; the outer task reads it
    // back at the end so we can hand the raw PCM to identify_user.
    let pcm_buf = Arc::new(Mutex::new(Vec::<u8>::with_capacity(
        (DEFAULT_AUDIO_SAMPLE_RATE as usize) * 2 * (max_seconds as usize),
    )));
    let pcm_buf_for_pump = Arc::clone(&pcm_buf);
    // Detect the mic's actual sample rate from the first chunk's
    // data size ÷ duration — no env var or proto change needed.
    let detected_rate: Arc<AtomicU32> = Arc::new(AtomicU32::new(0));
    let detected_rate_pump = Arc::clone(&detected_rate);
    let _language_owned = language.to_string();

    // Mic pump — drains the mic gRPC stream into the asr_req sender,
    // accumulates raw PCM, runs silence-VAD on each chunk, stops on
    // (whichever first):
    //   1. max_seconds hard ceiling
    //   2. mic stream EOF / error
    //   3. silence-VAD end-of-utterance (after speech then quiet)
    //   4. outer task drops asr_req_rx (server already returned FINAL)
    let pump_handle: tokio::task::JoinHandle<()> = tokio::spawn(async move {
        let deadline = tokio::time::Instant::now() + Duration::from_secs(max_seconds as u64);
        // [profile] mic transfer: t_pump=request start; first-chunk latency is
        // the audio-source startup (e.g. audio client bridge over the network); the
        // chunk count + total audio vs wall time shows transfer throughput.
        let t_pump = tokio::time::Instant::now();
        let no_speech_deadline = t_pump + Duration::from_secs(VAD_NO_SPEECH_TIMEOUT_SECS);
        let mut n_chunks: u32 = 0u32;
        let mut audio_s: f32 = 0.0;
        let mut logged_first = false;
        let mut has_spoken = false;
        let mut silence_secs: f32 = 0.0;
        let mut first_chunk = true;
        loop {
            let active_deadline = if has_spoken {
                deadline
            } else {
                deadline.min(no_speech_deadline)
            };
            if tokio::time::Instant::now() >= active_deadline {
                break;
            }
            let remaining = active_deadline - tokio::time::Instant::now();
            match tokio::time::timeout(remaining, mic_stream.message()).await {
                Ok(Ok(Some(chunk))) => {
                    if !logged_first {
                        info!(
                            "[profile] mic: first chunk +{} ms (audio-source startup / transfer)",
                            t_pump.elapsed().as_millis()
                        );
                        logged_first = true;
                    }
                    n_chunks += 1;
                    audio_s += chunk.duration_s.max(0.0);
                    let dur = chunk.duration_s.max(0.0);
                    // Detect sample rate from the first chunk:
                    // rate ≈ data_bytes / (duration_s × bytes_per_sample)
                    // For 16-bit mono: bytes_per_sample = 2.
                    if first_chunk && dur > 0.0 && chunk.data.len() >= 2 {
                        first_chunk = false;
                        let est = (chunk.data.len() as f64 / (dur as f64 * 2.0)).round() as u32;
                        if est >= 8000 {
                            detected_rate_pump.store(est, Ordering::Relaxed);
                        }
                    }
                    let rms = pcm_rms_s16le(&chunk.data);
                    if rms >= VAD_SPEECH_RMS {
                        has_spoken = true;
                        silence_secs = 0.0;
                    } else if has_spoken {
                        silence_secs += dur;
                    }
                    pcm_buf_for_pump.lock().await.extend_from_slice(&chunk.data);
                    let asr_chunk = crate::pb::asr::AsrAudioChunk { chunk: Some(chunk) };
                    if asr_req_tx.send(asr_chunk).await.is_err() {
                        break;
                    }
                    if has_spoken && silence_secs >= VAD_END_SILENCE_SECS {
                        // Drops asr_req_tx → server sees stream EOF →
                        // FunASR emits its is_final flush → outer loop
                        // collects the final transcript and breaks.
                        break;
                    }
                }
                Ok(Ok(None)) => break,
                Ok(Err(_)) => break,
                Err(_) => break,
            }
        }
        let wall = t_pump.elapsed().as_millis();
        info!(
            "[profile] mic: pump done +{wall} ms, {n_chunks} chunks, {audio_s:.2}s audio \
             (transfer realtime-factor {:.2}; >1 means audio arrived slower than realtime)",
            (wall as f32 / 1000.0) / audio_s.max(0.001)
        );
    });
    let _pump_abort_guard = AbortOnDrop(pump_handle.abort_handle());

    // FunASR docs: AsrAudioChunk + ASR backend reads its own audio_config
    // defaults (16 kHz mono pcm_s16le); language hint is on the bidi
    // request metadata, not the per-chunk message. Tonic-codegen
    // doesn't surface a separate header field here, so we just rely on
    // FunASR's auto-detection — paraformer-zh-streaming is zh-only so
    // this is fine. If a multi-lingual backend is ever wired in, the
    // contract needs an extra preamble message.
    let asr_resp = asr_client
        .recognize_stream(Request::new(asr_req_stream))
        .await
        .map_err(|e| anyhow::anyhow!("asr_stream rpc failed: {e}"))?;
    let mut asr_events = asr_resp.into_inner();

    // FunASR's paraformer-zh-streaming emits *incremental* partials —
    // each event carries only the words decoded from the most recent
    // chunk window, not a cumulative transcript. The is_final flush
    // (`recognize_chunk(b"", is_final=True)`) emits the residual
    // window's text the same way (sometimes empty).
    //
    // Strategy: append every non-empty event text into `accumulated`,
    // and use the accumulator as the final transcript. The server's
    // is_final flag only tells us when to stop the loop — it does
    // NOT mean "this event's text is the whole utterance"; treating
    // it that way drops every prior partial and leaves the user with
    // just the last syllable.
    let mut accumulated = String::new();
    let mut last_confidence = 0.0_f32;
    while let Some(ev_or_err) = asr_events.next().await {
        let ev = ev_or_err.map_err(|e| anyhow::anyhow!("asr_stream recv: {e}"))?;
        if !ev.error.is_empty() {
            anyhow::bail!("asr error: {}", ev.error);
        }
        let is_final = ev.is_final || ev.event_type == 1;
        let text = ev.text.clone();
        if !text.is_empty() {
            accumulated.push_str(&text);
            last_confidence = ev.confidence;
            if !is_final {
                let _ = tx
                    .send(Ok(event_text(
                        KIND_ASR_PARTIAL,
                        session_id,
                        &text,
                        ev.confidence,
                    )))
                    .await;
            }
        }
        if is_final {
            break;
        }
    }
    let transcript = accumulated;
    if !transcript.is_empty() {
        let _ = tx
            .send(Ok(event_text(
                KIND_ASR_FINAL,
                session_id,
                &transcript,
                last_confidence,
            )))
            .await;
    }

    // Stop the mic pump (the outer task already dropped asr_req_rx by
    // returning from the `while let` loop above, which closes the
    // request stream).
    pump_handle.abort();
    let _ = pump_handle.await;

    let pcm = std::mem::take(&mut *pcm_buf.lock().await);
    let sample_rate = detected_rate.load(Ordering::Relaxed);
    let sample_rate = if sample_rate > 0 {
        sample_rate
    } else {
        DEFAULT_AUDIO_SAMPLE_RATE
    };
    if let Some(path) = save_voice_recording(session_id, &pcm, sample_rate) {
        let _ = tx
            .send(Ok(event_status(
                KIND_RECORDING_DONE,
                session_id,
                &format!(
                    "saved voice recording to {} ({} bytes, ~{:.2}s @ {}Hz) ; transcript={:?}",
                    path.display(),
                    pcm.len(),
                    pcm.len() as f32 / (sample_rate as f32 * 2.0),
                    sample_rate,
                    transcript,
                ),
            )))
            .await;
    } else {
        let _ = tx
            .send(Ok(event_status(
                KIND_RECORDING_DONE,
                session_id,
                &format!(
                    "captured {} bytes (~{:.2}s @ {}Hz mono s16le); transcript={:?}",
                    pcm.len(),
                    pcm.len() as f32 / (sample_rate as f32 * 2.0),
                    sample_rate,
                    transcript,
                ),
            )))
            .await;
    }
    Ok((pcm, transcript))
}

// ── Voiceprint ───────────────────────────────────────────────────────────────

struct VoiceIdentity {
    response: Option<voiceprint::IdentifyResponse>,
}

struct AccessContext {
    method: String,
    confidence: f32,
    reason: String,
}

async fn identify_user(
    src: &Arc<Mutex<SrcClient>>,
    pin_provider_id: &str,
    audio_pcm: &[u8],
    fallback_hint: &str,
    session_id: &str,
    tx: &mpsc::Sender<Result<VoiceEvent, Status>>,
) -> VoiceIdentity {
    let fallback = if fallback_hint.is_empty() {
        "voice:unknown".to_string()
    } else if fallback_hint.starts_with("voice:") || fallback_hint.starts_with("local:") {
        fallback_hint.to_string()
    } else {
        format!("voice:{fallback_hint}")
    };

    let endpoint =
        match resolve_endpoint(src, "robonix/service/voiceprint/identify", pin_provider_id).await {
            Some(ep) => ep,
            None => {
                warn!("[voice] voiceprint provider unavailable; using hint {fallback}");
                let _ = tx
                    .send(Ok(event_user(
                        KIND_USER_IDENTIFIED,
                        session_id,
                        &fallback,
                        0.0,
                        "voiceprint provider unavailable",
                    )))
                    .await;
                return VoiceIdentity { response: None };
            }
        };

    let mut client = match RobonixServiceVoiceprintIdentifyClient::connect(endpoint.clone()).await {
        Ok(c) => c,
        Err(e) => {
            warn!("[voice] connect voiceprint at {endpoint}: {e}; using hint {fallback}");
            let _ = tx
                .send(Ok(event_user(
                    KIND_USER_IDENTIFIED,
                    session_id,
                    &fallback,
                    0.0,
                    "voiceprint connect failed",
                )))
                .await;
            return VoiceIdentity { response: None };
        }
    };

    let request = voiceprint::IdentifyRequest {
        audio_data: audio_pcm.to_vec(),
        encoding: DEFAULT_AUDIO_ENCODING.to_string(),
        sample_rate_hz: DEFAULT_AUDIO_SAMPLE_RATE,
    };

    match client.identify(Request::new(request)).await {
        Ok(resp) => {
            let resp = resp.into_inner();
            let user = if resp.user_id.is_empty() {
                fallback.clone()
            } else if resp.user_id.starts_with("voice:") || resp.user_id.starts_with("local:") {
                resp.user_id.clone()
            } else {
                format!("voice:{}", resp.user_id)
            };
            let detail = if resp.is_known {
                "voiceprint matched"
            } else {
                "voiceprint unknown speaker"
            };
            let _ = tx
                .send(Ok(event_user(
                    KIND_USER_IDENTIFIED,
                    session_id,
                    &user,
                    resp.confidence,
                    detail,
                )))
                .await;
            VoiceIdentity {
                response: Some(resp),
            }
        }
        Err(e) => {
            warn!("[voice] voiceprint RPC failed: {e}; using hint {fallback}");
            let _ = tx
                .send(Ok(event_user(
                    KIND_USER_IDENTIFIED,
                    session_id,
                    &fallback,
                    0.0,
                    "voiceprint rpc failed",
                )))
                .await;
            VoiceIdentity { response: None }
        }
    }
}

// ── TTS playback ─────────────────────────────────────────────────────────────

/// Prefer the speech service's complete playback capability, including native
/// robot TTS. Only absence permits PCM playback; a failed call is never retried
/// through another path because the robot may already have accepted the text.
async fn speak_text(
    src: &Arc<Mutex<SrcClient>>,
    tts_pin: &str,
    speaker_pin: &str,
    text: &str,
) -> Result<Option<String>> {
    const CONTRACT: &str = "robonix/service/speech/speak";
    let mut directory = src.lock().await;
    let providers = directory
        .query_capabilities("", CONTRACT, src_pb::Transport::Mcp)
        .await?;
    let Some(provider) = providers.iter().find(|provider| {
        (tts_pin.is_empty() || provider.id == tts_pin || provider.namespace == tts_pin)
            && provider.capabilities.iter().any(|cap| {
                cap.contract_id == CONTRACT && cap.transport == src_pb::Transport::Mcp as i32
            })
    }) else {
        return Ok(None);
    };
    let target = if speaker_pin.is_empty() {
        String::new()
    } else {
        directory
            .query_capabilities(
                "",
                "robonix/primitive/audio/speaker",
                src_pb::Transport::Grpc,
            )
            .await?
            .into_iter()
            .find(|speaker| speaker.id == speaker_pin || speaker.namespace == speaker_pin)
            .ok_or_else(|| anyhow::anyhow!("pinned speaker '{speaker_pin}' not in src"))?
            .id
    };
    let (channel_id, endpoint, _) = directory
        .connect_capability("gateway", &provider.id, CONTRACT, src_pb::Transport::Mcp)
        .await?;
    drop(directory);
    let result = tokio::time::timeout(Duration::from_secs(120), async {
        let transport =
            rmcp::transport::streamable_http_client::StreamableHttpClientTransport::from_uri(
                localhost_to_ipv4(&endpoint),
            );
        let mut client = ().serve(transport).await?;
        let response = client
            .call_tool(
                rmcp::model::CallToolRequestParams::new("speak").with_arguments(
                    serde_json::json!({"target": target, "text": text})
                        .as_object()
                        .expect("literal object")
                        .clone(),
                ),
            )
            .await;
        if let Err(error) = client.close().await {
            warn!("[voice] speech/speak MCP close failed: {error}");
        }
        let response = response?;
        let content = response
            .content
            .iter()
            .filter_map(|content| match &content.raw {
                rmcp::model::RawContent::Text(text) => Some(text.text.as_str()),
                _ => None,
            })
            .collect::<Vec<_>>()
            .join("\n");
        if response.is_error.unwrap_or(false) {
            anyhow::bail!("speech/speak failed: {content}");
        }
        let payload = match response.structured_content {
            Some(value) => value,
            None => serde_json::from_str(&content)?,
        };
        if payload.get("ok").and_then(serde_json::Value::as_bool) != Some(true) {
            anyhow::bail!("speech/speak rejected playback: {payload}");
        }
        Ok::<_, anyhow::Error>(
            payload
                .get("detail")
                .and_then(serde_json::Value::as_str)
                .unwrap_or("")
                .to_string(),
        )
    })
    .await
    .map_err(|_| anyhow::anyhow!("speech/speak timed out; playback acceptance unknown"))
    .and_then(|result| result);
    if let Err(error) = src.lock().await.disconnect_capability(&channel_id).await {
        warn!("[voice] speech/speak channel cleanup failed: {error}");
    }
    result.map(Some)
}

/// Play reply text through speech/speak when available, otherwise synthesize
/// PCM and stream it to the selected speaker. Reports acceptance via events.
async fn synthesize_and_play(
    src: &Arc<Mutex<SrcClient>>,
    tts_pin: &str,
    speaker_pin: &str,
    language: &str,
    text: &str,
    session_id: &str,
    tx: &mpsc::Sender<Result<VoiceEvent, Status>>,
) -> Result<()> {
    let _ = tx
        .send(Ok(event_status(
            KIND_TTS_STARTED,
            session_id,
            "speech playback requested",
        )))
        .await;
    if let Some(detail) = speak_text(src, tts_pin, speaker_pin, text).await? {
        let _ = tx
            .send(Ok(event_status(KIND_TTS_DONE, session_id, &detail)))
            .await;
        return Ok(());
    }
    let tts_endpoint = resolve_endpoint(src, "robonix/service/speech/tts", tts_pin)
        .await
        .ok_or_else(|| anyhow::anyhow!("no RobonixServiceSpeechTts provider"))?;
    let speaker_endpoint = resolve_endpoint(src, "robonix/primitive/audio/speaker", speaker_pin)
        .await
        .ok_or_else(|| anyhow::anyhow!("no RobonixPrimitiveAudioSpeaker provider"))?;

    let mut tts_client = RobonixServiceSpeechTtsClient::connect(tts_endpoint.clone())
        .await
        .map_err(|e| anyhow::anyhow!("connect tts {tts_endpoint}: {e}"))?;
    let resp = tts_client
        .synthesize(Request::new(tts::SynthesizeRequest {
            text: text.to_string(),
            language: language.to_string(),
            voice: String::new(),
            speed: 1.0,
        }))
        .await
        .map_err(|e| anyhow::anyhow!("tts rpc failed: {e}"))?
        .into_inner();
    if !resp.error.is_empty() {
        anyhow::bail!("tts error: {}", resp.error);
    }
    if resp.audio_data.is_empty() {
        anyhow::bail!("tts returned empty audio");
    }

    let mut speaker_client = RobonixPrimitiveAudioSpeakerClient::connect(speaker_endpoint.clone())
        .await
        .map_err(|e| anyhow::anyhow!("connect speaker {speaker_endpoint}: {e}"))?;
    const SLICE: usize = 8 * 1024;
    let chunks: Vec<AudioChunk> = resp
        .audio_data
        .chunks(SLICE)
        .enumerate()
        .map(|(i, slice)| AudioChunk {
            timestamp_ns: now_ns(),
            data: slice.to_vec(),
            sequence: i as u32,
            duration_s: 0.0,
        })
        .collect();
    let stream = futures::stream::iter(chunks);
    speaker_client
        .speaker(Request::new(stream))
        .await
        .map_err(|e| anyhow::anyhow!("speaker rpc failed: {e}"))?;

    let _ = tx
        .send(Ok(event_status(
            KIND_TTS_DONE,
            session_id,
            &format!(
                "played {} bytes via {speaker_endpoint}",
                resp.audio_data.len()
            ),
        )))
        .await;
    Ok(())
}

pub(crate) async fn play_prompt(
    src: &Arc<Mutex<SrcClient>>,
    tts_pin: &str,
    speaker_pin: &str,
    language: &str,
    text: &str,
    session_id: &str,
) -> Result<()> {
    let (tx, _rx) = mpsc::channel(2);
    synthesize_and_play(src, tts_pin, speaker_pin, language, text, session_id, &tx).await
}

// ── Helpers ──────────────────────────────────────────────────────────────────

fn build_task(
    session_id: &str,
    transcript: &str,
    user_id: &str,
    access: &AccessContext,
    audio_pcm: &[u8],
    extra_context_json: &str,
) -> Task {
    let mut ctx: serde_json::Value = if extra_context_json.trim().is_empty() {
        serde_json::json!({})
    } else {
        serde_json::from_str(extra_context_json).unwrap_or_else(|_| serde_json::json!({}))
    };
    if let Some(obj) = ctx.as_object_mut() {
        // Planner reads `modality` to decide brevity / no-markdown rules
        // for voice-mode replies (planner.rs:181). Don't rename without
        // updating planner in lockstep.
        obj.insert("modality".to_string(), serde_json::json!("voice"));
        obj.insert("voice_session".to_string(), serde_json::json!(true));
        obj.insert("user_id".to_string(), serde_json::json!(user_id));
        obj.insert(
            "access".to_string(),
            serde_json::json!({
                "allowed": true,
                "method": access.method,
                "confidence": access.confidence,
                "reason": access.reason,
            }),
        );
    }
    Task {
        task_id: Uuid::new_v4().to_string(),
        session_id: session_id.to_string(),
        source: crate::INTENT_SOURCE_AUDIO,
        text: transcript.to_string(),
        audio_data: audio_pcm.to_vec(),
        context_json: ctx.to_string(),
        timestamp_ms: now_ms(),
    }
}

fn context_flag(raw: &str, key: &str) -> bool {
    serde_json::from_str::<serde_json::Value>(raw)
        .ok()
        .and_then(|value| value.get(key).and_then(serde_json::Value::as_bool))
        .unwrap_or(false)
}

fn accumulate_text(ev: &PlannerEvent, into: &mut String) {
    const EVT_TEXT_CHUNK: u32 = 0;
    const EVT_FINAL_TEXT: u32 = 4;
    match ev.event_kind {
        EVT_TEXT_CHUNK => into.push_str(&ev.text_chunk),
        EVT_FINAL_TEXT if into.trim().is_empty() && !ev.final_text.is_empty() => {
            into.push_str(&ev.final_text);
        }
        _ => {}
    }
}

/// Pick the narration for a completed Planner stream segment.
///
/// FINAL_TEXT is authoritative: some providers emit it without preceding
/// TEXT_CHUNK events. Returning it here ensures voice sessions do not lose
/// their final spoken response while text sessions remain unaffected.
fn tts_boundary_text(kind: u32, streamed: String, final_text: &str) -> Option<String> {
    const EVT_FINAL_TEXT: u32 = 4;
    if kind == EVT_FINAL_TEXT && !final_text.trim().is_empty() {
        return Some(final_text.trim().to_string());
    }
    (!streamed.trim().is_empty()).then_some(streamed)
}

fn is_narration_boundary(kind: u32) -> bool {
    matches!(kind, 1..=4)
}

fn event_status(kind: u32, session_id: &str, message: &str) -> VoiceEvent {
    VoiceEvent {
        event_kind: kind,
        session_id: session_id.to_string(),
        text: String::new(),
        user_id: String::new(),
        confidence: 0.0,
        planner: None,
        error: String::new(),
        status_message: message.to_string(),
        timestamp_ms: now_ms(),
    }
}

fn event_text(kind: u32, session_id: &str, text: &str, confidence: f32) -> VoiceEvent {
    VoiceEvent {
        event_kind: kind,
        session_id: session_id.to_string(),
        text: text.to_string(),
        user_id: String::new(),
        confidence,
        planner: None,
        error: String::new(),
        status_message: String::new(),
        timestamp_ms: now_ms(),
    }
}

fn event_user(
    kind: u32,
    session_id: &str,
    user_id: &str,
    confidence: f32,
    status_message: &str,
) -> VoiceEvent {
    VoiceEvent {
        event_kind: kind,
        session_id: session_id.to_string(),
        text: String::new(),
        user_id: user_id.to_string(),
        confidence,
        planner: None,
        error: String::new(),
        status_message: status_message.to_string(),
        timestamp_ms: now_ms(),
    }
}

fn event_error(session_id: &str, error: &str) -> VoiceEvent {
    VoiceEvent {
        event_kind: KIND_ERROR,
        session_id: session_id.to_string(),
        text: String::new(),
        user_id: String::new(),
        confidence: 0.0,
        planner: None,
        error: error.to_string(),
        status_message: String::new(),
        timestamp_ms: now_ms(),
    }
}

fn is_mock_mode() -> bool {
    std::env::var("ROBONIX_GATEWAY_VOICE_MOCK")
        .map(|v| matches!(v.as_str(), "1" | "true" | "yes" | "on"))
        .unwrap_or(false)
        || std::env::var("ROBONIX_GATEWAY_VOICE_MOCK_TEXT").is_ok()
}

fn mock_transcript() -> String {
    std::env::var("ROBONIX_GATEWAY_VOICE_MOCK_TEXT")
        .unwrap_or_else(|_| "Hello, please introduce yourself.".to_string())
}

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

fn save_voice_recording(session_id: &str, pcm: &[u8], sample_rate: u32) -> Option<PathBuf> {
    let dump_dir = std::env::var("ROBONIX_GATEWAY_VOICE_SAVE_DIR")
        .ok()
        .filter(|s| !s.trim().is_empty())
        .unwrap_or_else(|| "/tmp".to_string());
    let dir = Path::new(&dump_dir);
    if fs::create_dir_all(dir).is_err() {
        return None;
    }
    let ts = now_ms();
    let file_name = format!("voice_{}_{}.wav", session_id, ts);
    let path = dir.join(file_name);
    if write_wav(&path, pcm, sample_rate).is_ok() {
        Some(path)
    } else {
        None
    }
}

fn write_wav(path: &Path, pcm: &[u8], sample_rate: u32) -> Result<(), std::io::Error> {
    const CHANNELS: u16 = 1;
    const BITS_PER_SAMPLE: u16 = 16;
    let data_size = pcm.len() as u32;
    let file_size = 36 + data_size;
    let byte_rate = sample_rate * CHANNELS as u32 * (BITS_PER_SAMPLE as u32) / 8;
    let block_align = CHANNELS * BITS_PER_SAMPLE / 8;

    let mut buf = Vec::with_capacity(44 + pcm.len());
    buf.extend_from_slice(b"RIFF");
    buf.extend_from_slice(&file_size.to_le_bytes());
    buf.extend_from_slice(b"WAVE");
    buf.extend_from_slice(b"fmt ");
    buf.extend_from_slice(&16u32.to_le_bytes());
    buf.extend_from_slice(&1u16.to_le_bytes());
    buf.extend_from_slice(&CHANNELS.to_le_bytes());
    buf.extend_from_slice(&sample_rate.to_le_bytes());
    buf.extend_from_slice(&byte_rate.to_le_bytes());
    buf.extend_from_slice(&block_align.to_le_bytes());
    buf.extend_from_slice(&BITS_PER_SAMPLE.to_le_bytes());
    buf.extend_from_slice(b"data");
    buf.extend_from_slice(&data_size.to_le_bytes());
    buf.extend_from_slice(pcm);

    fs::write(path, buf)
}

/// RMS amplitude of a 16-bit little-endian PCM buffer. Used by the
/// silence-VAD in the mic pump. Returns 0.0 on empty / odd-length input.
fn pcm_rms_s16le(data: &[u8]) -> f32 {
    let n = data.len() / 2;
    if n == 0 {
        return 0.0;
    }
    let sum_sq: f64 = data
        .as_chunks::<2>()
        .0
        .iter()
        .map(|b| i16::from_le_bytes([b[0], b[1]]) as f64)
        .map(|s| s * s)
        .sum();
    (sum_sq / n as f64).sqrt() as f32
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Run a real Src gRPC server on an ephemeral port for playback tests.
    async fn playback_src() -> (Arc<Mutex<SrcClient>>, AbortOnDrop) {
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let incoming = futures::stream::unfold(listener, |listener| async {
            Some((listener.accept().await.map(|(socket, _)| socket), listener))
        });
        let registry = robonix_src::service::SrcRegistry::with_contracts(Default::default());
        let server = robonix_src::service::SrcService::new(Arc::new(registry));
        let task = tokio::spawn(async move {
            tonic::transport::Server::builder()
                .add_service(src_pb::src_server::SrcServer::new(server))
                .serve_with_incoming(incoming)
                .await
                .unwrap();
        });
        let client = SrcClient::connect(format!("http://{address}"))
            .await
            .unwrap();
        (
            Arc::new(Mutex::new(client)),
            AbortOnDrop(task.abort_handle()),
        )
    }

    /// Serve MCP initialization and one configurable speak result over HTTP.
    /// Capture tool requests to verify selected hardware and detect retries.
    async fn playback_mcp(
        result: serde_json::Value,
    ) -> (String, mpsc::Receiver<serde_json::Value>, AbortOnDrop) {
        use tokio::io::{AsyncReadExt, AsyncWriteExt};
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let endpoint = format!("http://{}/mcp/", listener.local_addr().unwrap());
        let (calls, received) = mpsc::channel(8);
        let task = tokio::spawn(async move {
            loop {
                let (mut socket, _) = listener.accept().await.unwrap();
                let mut data = Vec::new();
                let header_end = loop {
                    let mut byte = [0];
                    socket.read_exact(&mut byte).await.unwrap();
                    data.push(byte[0]);
                    if data.ends_with(b"\r\n\r\n") {
                        break data.len();
                    }
                };
                let headers = String::from_utf8_lossy(&data);
                let length = headers
                    .lines()
                    .find_map(|line| {
                        let (name, value) = line.split_once(':')?;
                        name.eq_ignore_ascii_case("content-length")
                            .then(|| value.trim().parse::<usize>().unwrap())
                    })
                    .unwrap_or(0);
                let mut body = vec![0; length];
                socket.read_exact(&mut body).await.unwrap();
                let request: serde_json::Value = serde_json::from_slice(&body).unwrap_or_default();
                let response = match request["method"].as_str() {
                    Some("initialize") => serde_json::json!({
                        "jsonrpc": "2.0", "id": request["id"], "result": {
                            "protocolVersion": request["params"]["protocolVersion"],
                            "capabilities": {"tools": {}},
                            "serverInfo": {"name": "speech-test", "version": "1"}
                        }
                    }),
                    Some("tools/call") => {
                        calls.send(request.clone()).await.unwrap();
                        serde_json::json!({"jsonrpc": "2.0", "id": request["id"], "result": result})
                    }
                    _ => serde_json::Value::Null,
                };
                let (status, body) = if response.is_null() {
                    ("202 Accepted", String::new())
                } else {
                    ("200 OK", response.to_string())
                };
                let reply = format!(
                    "HTTP/1.1 {status}\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{body}",
                    body.len()
                );
                assert_eq!(header_end, data.len());
                socket.write_all(reply.as_bytes()).await.unwrap();
            }
        });
        (endpoint, received, AbortOnDrop(task.abort_handle()))
    }

    /// Exercise Src discovery and MCP wire calls without any robot playback.
    /// Native success needs no PCM capability; rejections must never retry PCM.
    #[tokio::test]
    async fn native_playback_uses_selected_speaker_and_reports_failures() {
        for (response, succeeds) in [
            (
                serde_json::json!({"content": [{"type": "text", "text": "{\"ok\":true,\"detail\":\"accepted\"}"}]}),
                true,
            ),
            (
                serde_json::json!({"content": [], "structuredContent": {"ok": true, "detail": "accepted"}}),
                true,
            ),
            (
                serde_json::json!({"content": [], "structuredContent": {"ok": false, "detail": "SDK rejected"}}),
                false,
            ),
            (
                serde_json::json!({"isError": true, "content": [{"type": "text", "text": "SDK failed"}]}),
                false,
            ),
        ] {
            let (src, _src_server) = playback_src().await;
            let (endpoint, mut calls, _mcp_server) = playback_mcp(response).await;
            {
                let mut directory = src.lock().await;
                directory
                    .register_service("speech", "robonix/service/speech", "")
                    .await
                    .unwrap();
                directory
                    .declare_capability(
                        "speech",
                        "robonix/service/speech/speak",
                        src_pb::Transport::Mcp,
                        &endpoint,
                        robonix_src::client::mcp_params("{}"),
                    )
                    .await
                    .unwrap();
                directory
                    .register_primitive("g1_audio", "robonix/primitive/audio", "")
                    .await
                    .unwrap();
                directory
                    .declare_capability(
                        "g1_audio",
                        "robonix/primitive/audio/speaker",
                        src_pb::Transport::Grpc,
                        "127.0.0.1:1",
                        robonix_src::client::grpc_params("", "", ""),
                    )
                    .await
                    .unwrap();
            }
            let (tx, mut events) = mpsc::channel(8);
            let result = tokio::time::timeout(
                Duration::from_secs(5),
                synthesize_and_play(
                    &src,
                    "speech",
                    "robonix/primitive/audio",
                    "zh-CN",
                    "你好",
                    "test",
                    &tx,
                ),
            )
            .await
            .unwrap();
            assert_eq!(result.is_ok(), succeeds, "{result:?}");
            if !succeeds {
                assert!(result.unwrap_err().to_string().contains("speech/speak"));
            }
            let call = calls.recv().await.unwrap();
            assert_eq!(call["params"]["name"], "speak");
            assert_eq!(
                call["params"]["arguments"],
                serde_json::json!({"target": "g1_audio", "text": "你好"})
            );
            assert!(calls.try_recv().is_err());
            assert_eq!(
                events.recv().await.unwrap().unwrap().event_kind,
                KIND_TTS_STARTED
            );
            if succeeds {
                assert_eq!(
                    events.recv().await.unwrap().unwrap().event_kind,
                    KIND_TTS_DONE
                );
            }
            assert!(events.try_recv().is_err());
        }
    }

    /// Preserve the PCM branch only when the selected service lacks speak.
    #[tokio::test]
    async fn absent_speak_capability_preserves_pcm_path() {
        let (src, _server) = playback_src().await;
        assert_eq!(
            speak_text(&src, "missing", "g1_audio", "你好")
                .await
                .unwrap(),
            None
        );
        let (tx, _events) = mpsc::channel(8);
        let error = synthesize_and_play(&src, "missing", "g1_audio", "", "你好", "test", &tx)
            .await
            .unwrap_err();
        assert!(
            error
                .to_string()
                .contains("no RobonixServiceSpeechTts provider")
        );
    }

    #[test]
    fn fallback_user_id_normalises_hint() {
        let hint = "liukaile";
        let normalised = if hint.is_empty() {
            "voice:unknown".to_string()
        } else if hint.starts_with("voice:") || hint.starts_with("local:") {
            hint.to_string()
        } else {
            format!("voice:{hint}")
        };
        assert_eq!(normalised, "voice:liukaile");
    }

    #[test]
    fn build_task_injects_user_id_into_context() {
        let task = build_task(
            "sess-1",
            "hello",
            "voice:alice",
            &AccessContext {
                method: "voiceprint".to_string(),
                confidence: 0.9,
                reason: "matched".to_string(),
            },
            &[],
            r#"{"foo":"bar"}"#,
        );
        let v: serde_json::Value = serde_json::from_str(&task.context_json).unwrap();
        assert_eq!(v["user_id"], "voice:alice");
        assert_eq!(v["voice_session"], true);
        assert_eq!(v["access"]["allowed"], true);
        assert_eq!(v["access"]["method"], "voiceprint");
        assert_eq!(v["foo"], "bar");
    }

    #[test]
    fn accumulate_text_collects_chunks_and_final() {
        let mut buf = String::new();
        accumulate_text(
            &PlannerEvent {
                event_kind: 0,
                session_id: "s".into(),
                text_chunk: "hello ".into(),
                plan: None,
                batch_result: None,
                status: None,
                final_text: String::new(),
                node_state: None,
                task_state: None,
            },
            &mut buf,
        );
        accumulate_text(
            &PlannerEvent {
                event_kind: 0,
                session_id: "s".into(),
                text_chunk: "world".into(),
                plan: None,
                batch_result: None,
                status: None,
                final_text: String::new(),
                node_state: None,
                task_state: None,
            },
            &mut buf,
        );
        assert_eq!(buf, "hello world");
    }

    #[test]
    fn final_text_is_spoken_without_text_chunks() {
        assert_eq!(
            tts_boundary_text(4, String::new(), "final answer"),
            Some("final answer".to_string())
        );
    }

    #[test]
    fn status_flushes_progress_without_requiring_final_text() {
        assert!(is_narration_boundary(3));
        assert_eq!(
            tts_boundary_text(3, "still navigating".to_string(), ""),
            Some("still navigating".to_string())
        );
    }

    #[test]
    fn barge_in_metadata_is_explicit_not_inferred_from_voice_mode() {
        assert!(context_flag(r#"{"barge_in":true}"#, "barge_in"));
        assert!(!context_flag(r#"{"interaction_mode":"voice"}"#, "barge_in"));
        assert!(!context_flag("not json", "barge_in"));
    }

    #[tokio::test]
    async fn dropping_voice_stream_aborts_session_task() {
        let (_tx, rx) = mpsc::channel(1);
        let task = tokio::spawn(std::future::pending::<()>());
        let abort = task.abort_handle();
        let stream = VoiceSessionStream {
            inner: ReceiverStream::new(rx),
            task,
        };
        drop(stream);
        tokio::task::yield_now().await;
        assert!(abort.is_finished());
    }

    #[tokio::test]
    async fn dropping_abort_guard_aborts_nested_mic_pump() {
        let task = tokio::spawn(std::future::pending::<()>());
        let abort = task.abort_handle();
        drop(AbortOnDrop(abort.clone()));
        tokio::task::yield_now().await;
        assert!(abort.is_finished());
        let _ = task.await;
    }
}
