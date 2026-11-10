// SPDX-License-Identifier: MulanPSL-2.0
// Tonic-generated wire types for every proto in planner's import closure.
//
// `contract_proto_modules.rs` is emitted by robonix-codegen (build.rs) and
// declares `pub mod <pkg> { tonic::include_proto!(...); }` for each proto
// package, ordered so that prost `super::sibling` references resolve.
//
// All cross-call proto types planner uses live under here:
//   pb::contracts        — RobonixSystemPlanner / RobonixSystemExecutorExecute / …
//   pb::planner          — Task / Plan / CapabilityCall / PlannerEvent / …
//   pb::executor         — RtdlEvent / ListTools_Request / CapabilitySpec / …
//   pb::robonix_msg      — ChatMessage / ChatPart / ToolCall / CapabilitySpec
//   pb::std_msgs         — String / Empty / …
//
// The codegen output triggers a lot of dead_code / unused warnings on
// types planner itself never references; silence them here so `cargo build`
// stays readable.
#![allow(
    dead_code,
    unused_imports,
    unused_variables,
    clippy::all,
    rustdoc::broken_intra_doc_links,
    rustdoc::invalid_html_tags
)]

include!(concat!(env!("OUT_DIR"), "/contract_proto_modules.rs"));
