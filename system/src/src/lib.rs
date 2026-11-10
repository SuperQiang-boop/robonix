// SPDX-License-Identifier: MulanPSL-2.0
// Robonix Src: control plane for Provider registration, Capability
// discovery, and gRPC channel negotiation.
//
// - `service` module: server-side gRPC handlers and the in-memory
//   `SrcRegistry`. The `robonix-src` binary glues these together.
// - `client` module: thin client-side helpers (Register* / Declare /
//   Query / Connect) shared by every Robonix component that talks to
//   Src (planner, executor, cli, services, …).
// - `pb` module: tonic-generated wire types and stubs.

extern crate self as robonix_src;

pub mod pb {
    tonic::include_proto!("robonix.src");
}

pub mod client;
pub mod contract_registry;
pub mod service;
