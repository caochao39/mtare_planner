// Copyright 2010-2018 Google LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OR_TOOLS_UTIL_FILE_UTIL_H_
#define OR_TOOLS_UTIL_FILE_UTIL_H_

#include <limits>
#include <vector>

#include "absl/strings/string_view.h"
#include "google/protobuf/message.h"
#include "ortools/base/file.h"
#include "ortools/base/recordio.h"
#include "ortools/base/statusor.h"

namespace operations_research {

// Reads a file, optionally gzipped, to a std::string.
util::StatusOr<std::string> ReadFileToString(absl::string_view filename);

// Reads a proto from a file. Supports the following formats: binary, text,
// JSON, all of those optionally gzipped. Returns false on failure.
bool ReadFileToProto(absl::string_view filename,
                     google::protobuf::Message* proto);

template <typename Proto>
Proto ReadFileToProtoOrDie(absl::string_view filename) {
  Proto proto;
  CHECK(ReadFileToProto(filename, &proto));
  return proto;
}

enum class ProtoWriteFormat { kProtoText, kProtoBinary, kJson };

// Writes a proto to a file. Supports the following formats: binary, text, JSON,
// all of those optionally gzipped. Returns false on failure.
// If 'proto_write_format' is kProtoBinary, ".bin" is appended to file_name. If
// 'proto_write_format' is kJson, ".json" is appended to file_name. If 'gzipped'
// is true, ".gz" is appended to file_name.
bool WriteProtoToFile(absl::string_view filename,
                      const google::protobuf::Message& proto,
                      ProtoWriteFormat proto_write_format, bool gzipped);

namespace internal {
// General method to read expected_num_records from a file. If
// expected_num_records is -1, then reads all records from the file. If not,
// dies if the file doesn't contain exactly expected_num_records.
template <typename Proto>
std::vector<Proto> ReadNumRecords(File* file, int expected_num_records) {
  recordio::RecordReader reader(file);
  std::vector<Proto> protos;
  Proto proto;
  int num_read = 0;
  while (num_read != expected_num_records &&
         reader.ReadProtocolMessage(&proto)) {
    protos.push_back(proto);
    ++num_read;
  }

  if (expected_num_records >= 0) {
    CHECK_EQ(num_read, expected_num_records)
        << "There were less than the expected " << expected_num_records
        << " in the file.";
  }

  return protos;
}

// Ditto, taking a filename as argument.
template <typename Proto>
std::vector<Proto> ReadNumRecords(absl::string_view filename,
                                  int expected_num_records) {
  return ReadNumRecords<Proto>(file::OpenOrDie(filename, "r", file::Defaults()),
                               expected_num_records);
}
}  // namespace internal

// Reads all records in Proto format in 'file'. Silently does nothing if the
// file is empty. Dies if the file doesn't exist or contains something else than
// protos encoded in RecordIO format.
template <typename Proto>
std::vector<Proto> ReadAllRecordsOrDie(absl::string_view filename) {
  return internal::ReadNumRecords<Proto>(filename, -1);
}
template <typename Proto>
std::vector<Proto> ReadAllRecordsOrDie(File* file) {
  return internal::ReadNumRecords<Proto>(file, -1);
}

// Reads one record in Proto format in 'file'. Dies if the file doesn't exist,
// doesn't contain exactly one record, or contains something else than protos
// encoded in RecordIO format.
template <typename Proto>
Proto ReadOneRecordOrDie(absl::string_view filename) {
  Proto p;
  p.Swap(&internal::ReadNumRecords<Proto>(filename, 1)[0]);
  return p;
}

// Writes all records in Proto format to 'file'. Dies if it is unable to open
// the file or write to it.
template <typename Proto>
void WriteRecordsOrDie(absl::string_view filename,
                       const std::vector<Proto>& protos) {
  recordio::RecordWriter writer(
      file::OpenOrDie(filename, "w", file::Defaults()));
  for (const Proto& proto : protos) {
    CHECK(writer.WriteProtocolMessage(proto));
  }
}

}  // namespace operations_research

#endif  // OR_TOOLS_UTIL_FILE_UTIL_H_
