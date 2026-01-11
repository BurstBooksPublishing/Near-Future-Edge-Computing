#include 
#include 
#include 
#include 
#include 

int main() {
  leveldb::Options options;
  options.create_if_missing = true;
  options.compression = leveldb::kSnappyCompression; // reduce space, CPU trade-off
  options.write_buffer_size = 4 << 20; // 4MB memtable for constrained RAM
  options.max_open_files = 500;

  leveldb::DB* raw_db = nullptr;
  leveldb::Status s = leveldb::DB::Open(options, "/data/leveldb", &raw_db);
  if (!s.ok()) { std::cerr << "Open error: " << s.ToString() << "\n"; return 2; }
  std::unique_ptr db(raw_db);

  // Atomic batch write for telemetry events
  leveldb::WriteBatch batch;
  batch.Put("sensor:123:ts:1600000000", "payloadA");
  batch.Put("sensor:124:ts:1600000001", "payloadB");
  s = db->Write(leveldb::WriteOptions(), &batch); // non-sync for throughput
  if (!s.ok()) std::cerr << "Write error: " << s.ToString() << "\n";

  // Snapshot read for consistent view across reads
  const leveldb::Snapshot* snap = db->GetSnapshot();
  leveldb::ReadOptions rdopts;
  rdopts.snapshot = snap;
  std::string value;
  s = db->Get(rdopts, "sensor:123:ts:1600000000", &value);
  if (s.ok()) std::cout << "Read: " << value << "\n";
  db->ReleaseSnapshot(snap);

  // Range scan: process latest N keys by iterator
  leveldb::Iterator* it = db->NewIterator(leveldb::ReadOptions());
  for (it->SeekToLast(); it->Valid(); it->Prev()) {
    // process it->key() and it->value()
    // break after desired window
  }
  if (!it->status().ok()) std::cerr << "Iterator error: " << it->status().ToString() << "\n";
  delete it;

  // Explicit background compaction for critical maintenance
  db->CompactRange(nullptr, nullptr); // compact entire keyspace during maintenance window

  return 0;
}