// SPDX-License-Identifier: MulanPSL-2.0
// Tonic-generated wire types. `contract_proto_modules.rs` is emitted by
// robonix-codegen (build.rs) and declares `pub mod <pkg>` for each proto
// package, ordered so prost `super::sibling` references resolve.
//
// Used here:
//   pb::contracts::robonix_system_monitor_get_server    — GetMonitor RPC handler
//   pb::contracts::robonix_system_monitor_stream_server  — StreamMonitor server_stream handler
//   pb::monitor                                          — MonitorSnapshot, PowerState, ComponentHealth
#![allow(
    dead_code,
    unused_imports,
    unused_variables,
    clippy::all,
    rustdoc::broken_intra_doc_links,
    rustdoc::invalid_html_tags
)]

include!(concat!(env!("OUT_DIR"), "/contract_proto_modules.rs"));
