from bcc import BPF
import socket, struct, sys

# C XDP program: checks src IP against map and drops on match.
prog = r"""
#include 
#include 
#include 

BPF_HASH(blacklist, u32, u8); // map of blacklisted IPv4 addresses

int xdp_ips(struct xdp_md *ctx) {
    void *data = (void *)(long)ctx->data;
    void *data_end = (void *)(long)ctx->data_end;
    struct ethhdr *eth = data;
    if ((void*)eth + sizeof(*eth) > data_end) return XDP_PASS;
    if (eth->h_proto != htons(ETH_P_IP)) return XDP_PASS;

    struct iphdr *ip = data + sizeof(*eth);
    if ((void*)ip + sizeof(*ip) > data_end) return XDP_PASS;

    u32 src = ip->saddr;
    u8 *val = blacklist.lookup(&src);
    if (val) return XDP_DROP; // drop if blacklisted
    return XDP_PASS;
}
"""

if len(sys.argv) != 2:
    print("Usage: python3 loader.py ")
    sys.exit(1)
iface = sys.argv[1]

b = BPF(text=prog)
fn = b.load_func("xdp_ips", BPF.XDP)
b.attach_xdp(iface, fn, 0)  # attach XDP to interface

# Helper: add IP to blacklist map from controller (illustrative).
def ip_to_u32(ip):
    return struct.unpack("!I", socket.inet_aton(ip))[0]

def add_blacklist(ip):
    key = struct.pack("I", socket.htonl(ip_to_u32(ip)))
    b["blacklist"][key] = b"\x01"  # value is a single byte

# Example: add an address (production would use gRPC/MQTT control plane)
add_blacklist("192.0.2.10")
print("XDP IPS attached on", iface, "- blacklist updated")
# Run until interrupted; cleanup handled by signal in production code.