#pragma once

#include <cstdint>
#include <iosfwd>
#include <string>
#include <utility>
#include <vector>

namespace isocity {

// Minimal JSON value representation and parser.
//
// Why is this in-core?
//  - Keep headless tools dependency-free (no third-party JSON libs).
//  - Provide a stable way to load/save ProcGenConfig / SimConfig overrides.
//
// Notes:
//  - Strict JSON: no comments, no trailing commas.
//  - Numbers are parsed as double.
//  - Objects are stored as an ordered list of key/value pairs.
//
struct JsonValue {
  enum class Type : std::uint8_t {
    Null,
    Bool,
    Number,
    String,
    Array,
    Object,
  };

  Type type = Type::Null;

  bool boolValue = false;
  double numberValue = 0.0;
  std::string stringValue;
  std::vector<JsonValue> arrayValue;
  std::vector<std::pair<std::string, JsonValue>> objectValue;

  static JsonValue MakeNull();
  static JsonValue MakeBool(bool b);
  static JsonValue MakeNumber(double n);
  static JsonValue MakeString(std::string s);
  static JsonValue MakeArray();
  static JsonValue MakeObject();

  bool isNull() const { return type == Type::Null; }
  bool isBool() const { return type == Type::Bool; }
  bool isNumber() const { return type == Type::Number; }
  bool isString() const { return type == Type::String; }
  bool isArray() const { return type == Type::Array; }
  bool isObject() const { return type == Type::Object; }
};

const JsonValue* FindJsonMember(const JsonValue& obj, const std::string& key);
JsonValue* FindJsonMember(JsonValue& obj, const std::string& key);

bool ParseJson(const std::string& text, JsonValue& outValue, std::string& outError);

// Escape a string to be used inside a JSON string literal (without surrounding quotes).
std::string JsonEscape(const std::string& s);

struct JsonWriteOptions {
  // Pretty-print with newlines + indentation.
  bool pretty = true;

  // Spaces per indentation level when pretty-printing.
  int indent = 2;

  // Sort object keys lexicographically (useful for deterministic outputs).
  bool sortKeys = false;
};

// Serialize a JsonValue to a stream.
//
// Returns false on non-finite numbers (NaN/Inf) or stream failures.
bool WriteJson(std::ostream& os, const JsonValue& value, std::string& outError,
              const JsonWriteOptions& opt = {});

// Serialize a JsonValue to a string.
std::string JsonStringify(const JsonValue& value, const JsonWriteOptions& opt = {});

// Convenience: write a JSON file.
bool WriteJsonFile(const std::string& path, const JsonValue& value, std::string& outError,
                   const JsonWriteOptions& opt = {});



// -----------------------------------------------------------------------------------------------
// JsonWriter
//
// Streaming JSON writer for deterministic, dependency-free JSON output.
//
// Why:
//  - Many exporters (GeoJSON, transit plans, graphs) want to stream large JSON without building a
//    full JsonValue tree.
//  - Hand-rolled JSON string concatenation is error-prone (escaping, commas, indentation).
//
// Notes:
//  - JsonWriter does NOT sort object keys automatically; callers control key order.
//  - On misuse, JsonWriter stores an error message and subsequent calls return false.
// -----------------------------------------------------------------------------------------------
class JsonWriter {
public:
  explicit JsonWriter(std::ostream& os, JsonWriteOptions opt = {});

  // Reset writer state (does not clear the underlying stream).
  void reset();

  bool ok() const { return m_error.empty(); }
  const std::string& error() const { return m_error; }

  const JsonWriteOptions& options() const { return m_opt; }
  void setOptions(const JsonWriteOptions& opt) { m_opt = opt; }

  // Containers (objects/arrays).
  bool beginObject();
  bool endObject();
  bool beginArray();
  bool endArray();

  // Object member key (must be inside an object).
  bool key(const std::string& k);

  // Primitive values.
  bool nullValue();
  bool boolValue(bool b);
  bool numberValue(double n);
  bool intValue(std::int64_t n);
  bool uintValue(std::uint64_t n);
  bool stringValue(const std::string& s);

  // Convenience: serialize a JsonValue as a compact subtree (pretty=false) in the current context.
  bool value(const JsonValue& v);

private:
  struct Frame {
    enum class Kind : std::uint8_t {
      Object,
      Array,
    };

    Kind kind = Kind::Object;

    // For arrays: whether the next element is the first.
    // For objects: whether the next member is the first.
    bool first = true;

    // Only meaningful for objects: true when the next operation must be key().
    bool expectingKey = true;
  };

  bool setError(std::string msg);

  bool writeChar(char c);
  bool writeRaw(const char* s);
  bool writeRaw(const std::string& s);

  void indent(int depth);

  // Prepare for writing a value in the current context (handles commas + indentation).
  bool prepareValue();

  // Mark completion of a value (updates parent frame state).
  bool finishValue();

  bool beginContainer(Frame::Kind kind, char openChar);
  bool endContainer(Frame::Kind kind, char closeChar);

  std::ostream* m_os = nullptr;
  JsonWriteOptions m_opt{};
  std::vector<Frame> m_stack;
  bool m_finished = false;
  std::string m_error;
};


} // namespace isocity
