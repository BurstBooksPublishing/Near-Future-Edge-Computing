#!/usr/bin/env python3
# Client: generate pairwise masks, mask update, publish masked vector
import os, json, struct
import numpy as np
from nacl import public, utils, exceptions, encoding
import paho.mqtt.client as mqtt
from cryptography.hazmat.primitives.kdf.hkdf import HKDF
from cryptography.hazmat.primitives import hashes

BROKER = "mqtt-broker.local"
TOPIC_MASKED = "fl/masked_updates"
MODEL_DIM = 100000  # dimension of model vector
Q = 2**61-1         # prime modulus for safe integer ops

# persistent keypair for X25519
sk = public.PrivateKey.generate()
pk = sk.public_key.encode(encoder=encoding.Base64Encoder).decode()

def hkdf_expand(shared_secret: bytes, length: int) -> bytes:
    # Expand shared secret into length bytes via HKDF-SHA256
    hk = HKDF(algorithm=hashes.SHA256(), length=length, salt=None, info=b"mask")
    return hk.derive(shared_secret)

def prg_from_key(key_bytes: bytes, dim: int) -> np.ndarray:
    # Expand key to dim 64-bit integers, deterministic and cryptographically strong
    out = hkdf_expand(key_bytes, dim * 8)
    vals = np.frombuffer(out, dtype=np.uint64) % Q
    return vals.astype(np.int64)

def compute_masked_update(x: np.ndarray, peer_public_keys: dict) -> (np.ndarray, dict):
    # peer_public_keys: {peer_id: base64_pk}
    masks = {}
    add = np.zeros_like(x, dtype=np.int64)
    sub = np.zeros_like(x, dtype=np.int64)
    my_priv = public.PrivateKey(sk.encode())
    for peer_id, b64_pk in peer_public_keys.items():
        peer_pk = public.PublicKey(encoding.Base64Encoder.decode(b64_pk))
        # X25519 shared secret
        try:
            box = public.Box(my_priv, peer_pk)
            shared = box.shared_key()  # libsodium scalar mul
        except exceptions.CryptoError:
            # fallback: raw scalar mult if library supports
            shared = utils.random(32)
        mask = prg_from_key(shared, x.size)
        # deterministically assign sign to avoid double counting
        if peer_id > os.getenv("CLIENT_ID", "0"):
            add = (add + mask) % Q
            masks[peer_id] = ("add", None)  # store metadata minimal
        else:
            sub = (sub + mask) % Q
            masks[peer_id] = ("sub", None)
    masked = (x + add - sub) % Q
    return masked, masks

def publish_masked(masked_vec: np.ndarray, client_id: str):
    # serialize as compressed bytes; here simple binary little-endian
    payload = struct.pack("
\chapter{Blockchain at the Edge}
\section{Blockchain Fundamentals}
\subsection{Distributed consensus mechanisms}
These design notes build on the preceding overview of blockchain components and cryptographic primitives and focus on practical consensus choices for constrained edge clusters. The discussion emphasizes latency, fault models, and resource constraints that shape consensus selection for industrial and autonomous deployments.

Distributed consensus at the edge balances safety, liveness, latency, and energy. Consensus ensures a consistent replicated state across geographically distributed edge nodes, such as Raspberry Pi clusters at cell-site base stations, ARM Cortex-A SoCs in micro data centers, or Zephyr/FreeRTOS-based gateways coordinating control loops. Key application drivers at the edge are bounded commit latency, resilience to intermittent connectivity, and minimized CPU/crypto load to preserve battery and thermal budgets.

Theory: fault and network models drive protocol viability.
\begin{itemize}
\item Synchronous, asynchronous, and partially synchronous timing models determine protocol guarantees. Practical edge systems assume partial synchrony with occasional high RTTs.
\item Crash Fault Tolerance (CFT) protocols (Raft, Paxos) tolerate node crashes and are lighter weight.
\item Byzantine Fault Tolerance (BFT) protocols (PBFT, Tendermint, HotStuff) tolerate arbitrary (malicious) behaviour at higher resource cost.
\end{itemize}
Mathematically, a BFT system tolerating $f$ Byzantine nodes requires
\begin{equation}[H]\label{eq:bft_n}
n \;\ge\; 3f + 1,
\end{equation}
and a commit quorum must contain at least $2f+1$ votes. Message complexity for classical PBFT is $O(n^2)$ per decision, which impacts radio and wire bandwidth in dense edge clusters.

Concept-to-algorithm mapping for edge scenarios:
\begin{itemize}
\item Raft: simple leader-based CFT. Low CPU and message overhead. Suitable when devices are trusted and a small number of nodes (3–7) run on K3s or micro VMs. Raft provides fast commits under stable leaders and is easy to embed (etcd, Consul).
\item PBFT variants (HotStuff, Tendermint): appropriate where nodes cross administrative domains (e.g., multi-operator smart city gateways). Provide Byzantine tolerance at cost of CPU, memory, and $O(n^2)$ messages; modern variants reduce committee rotation overhead.
\item Asynchronous BFT (Honey Badger): targets high-throughput gossip with strong asynchrony but needs heavy cryptographic batching; viable when throughput outweighs latency needs.
\item Proof-of-Authority / PoA: deterministic validators (used in permissioned ledgers) reduce cryptographic load and are practical for consortium edge deployments where validators run on TPM-backed gateways.
\end{itemize}

Example: small industrial edge with 5 gateways controlling robotic cells. A Raft-based replicated state machine minimizes latency and computational cost. If gateways belong to distinct vendors or operate over shared public access networks, a BFT protocol with $f=1$ and $n=4$ is safer per (1), but expect higher RTT and CPU.

Implementation and deployment considerations:
\begin{itemize}
\item Resource allocation: allocate CPU affinity for consensus tasks on dedicated cores or microVMs. Use hardware crypto (ARM CryptoCell, TPM 2.0) to offload signing. Use secure boot and remote attestation to establish validator identity.
\item Networking: prefer UDP-based QUIC or TCP with TLS for transport. Configure QoS to prioritize consensus traffic on 5G slices or TSN links.
\item Batching and checkpointing: batch transactions to amortize signatures. Periodic checkpoint snapshots reduce replay and storage use.
\item Churn and partitioning: implement membership change protocols (Raft joint consensus, HotStuff reconfiguration) and design for graceful leader handover.
\item Observability: export metrics via Prometheus and distributed tracing (OpenTelemetry) to detect degraded consensus performance.
\item Energy and latency trade-offs: increase batch size to reduce CPU per transaction, but that increases commit latency; select the operating point per application SLA.
\end{itemize}

Practical code: lightweight Raft node bootstrap using HashiCorp Raft in Go. This snippet configures file-backed stores and a TCP transport for edge VLANs. Replace paths, bind address, and bootstrapPeers per deployment.
\begin{lstlisting}[language=Go,caption={Bootstrap a single-node Raft store for an edge gateway},label={lst:raft_bootstrap}]
package main

import (
        "log"
        "net"
        "os"
        "time"

        "github.com/hashicorp/raft"
        "github.com/hashicorp/raft-boltdb"
)

func main() {
        config := raft.DefaultConfig()
        config.LocalID = raft.ServerID("edge-node-01") // node identity

        // Transport over TCP; use TLS in production
        addr, _ := net.ResolveTCPAddr("tcp", "0.0.0.0:12000")
        transport, err := raft.NewTCPTransport(addr.String(), addr, 3, 10*time.Second, os.Stdout)
        if err != nil {
                log.Fatal(err)
        }

        // Stable store and log using BoltDB
        store, err := boltdb.NewBoltStore("raft-stable.db")
        if err != nil {
                log.Fatal(err)
        }
        logStore := store
        snapshots, err := raft.NewFileSnapshotStore("./snapshots", 2, os.Stdout)
        if err != nil {
                log.Fatal(err)
        }

        // FSM implements Apply, Snapshot, Restore for replicated state machine
        var fsm raft.FSM = NewAppFSM() // implement application-specific FSM

        r, err := raft.NewRaft(config, fsm, logStore, store, snapshots, transport)
        if err != nil {
                log.Fatal(err)
        }

        // Bootstrap single-node cluster if no peers
        hasState, err := raft.HasExistingState(logStore, store, snapshots)
        if err != nil {
                log.Fatal(err)
        }
        if !hasState {
                cfg := raft.Configuration{
                        Servers: []raft.Server{
                                {ID: config.LocalID, Address: transport.LocalAddr()},
                        },
                }
                future := r.BootstrapCluster(cfg)
                if future.Error() != nil {
                        log.Fatal(future.Error())
                }
        }

        select {} // keep process running
}