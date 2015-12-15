// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Messages/Geometry/Vector3.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "Messages/Geometry/Vector3.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace MaxBotMessage {

namespace {

const ::google::protobuf::Descriptor* Vector3_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  Vector3_reflection_ = NULL;
const ::google::protobuf::Descriptor* Vector3Stamped_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  Vector3Stamped_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_Messages_2fGeometry_2fVector3_2eproto() {
  protobuf_AddDesc_Messages_2fGeometry_2fVector3_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "Messages/Geometry/Vector3.proto");
  GOOGLE_CHECK(file != NULL);
  Vector3_descriptor_ = file->message_type(0);
  static const int Vector3_offsets_[3] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Vector3, x_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Vector3, y_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Vector3, z_),
  };
  Vector3_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      Vector3_descriptor_,
      Vector3::default_instance_,
      Vector3_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Vector3, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Vector3, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(Vector3));
  Vector3Stamped_descriptor_ = file->message_type(1);
  static const int Vector3Stamped_offsets_[2] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Vector3Stamped, stamp_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Vector3Stamped, vector_),
  };
  Vector3Stamped_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      Vector3Stamped_descriptor_,
      Vector3Stamped::default_instance_,
      Vector3Stamped_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Vector3Stamped, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Vector3Stamped, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(Vector3Stamped));
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_Messages_2fGeometry_2fVector3_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    Vector3_descriptor_, &Vector3::default_instance());
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    Vector3Stamped_descriptor_, &Vector3Stamped::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_Messages_2fGeometry_2fVector3_2eproto() {
  delete Vector3::default_instance_;
  delete Vector3_reflection_;
  delete Vector3Stamped::default_instance_;
  delete Vector3Stamped_reflection_;
}

void protobuf_AddDesc_Messages_2fGeometry_2fVector3_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::MaxBotMessage::protobuf_AddDesc_Messages_2fStamp_2eproto();
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\037Messages/Geometry/Vector3.proto\022\rMaxBo"
    "tMessage\032\024Messages/Stamp.proto\"*\n\007Vector"
    "3\022\t\n\001x\030\001 \002(\001\022\t\n\001y\030\002 \002(\001\022\t\n\001z\030\003 \002(\001\"]\n\016Ve"
    "ctor3Stamped\022#\n\005stamp\030\001 \002(\0132\024.MaxBotMess"
    "age.Stamp\022&\n\006vector\030\002 \002(\0132\026.MaxBotMessag"
    "e.Vector3", 209);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "Messages/Geometry/Vector3.proto", &protobuf_RegisterTypes);
  Vector3::default_instance_ = new Vector3();
  Vector3Stamped::default_instance_ = new Vector3Stamped();
  Vector3::default_instance_->InitAsDefaultInstance();
  Vector3Stamped::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_Messages_2fGeometry_2fVector3_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_Messages_2fGeometry_2fVector3_2eproto {
  StaticDescriptorInitializer_Messages_2fGeometry_2fVector3_2eproto() {
    protobuf_AddDesc_Messages_2fGeometry_2fVector3_2eproto();
  }
} static_descriptor_initializer_Messages_2fGeometry_2fVector3_2eproto_;

// ===================================================================

#ifndef _MSC_VER
const int Vector3::kXFieldNumber;
const int Vector3::kYFieldNumber;
const int Vector3::kZFieldNumber;
#endif  // !_MSC_VER

Vector3::Vector3()
  : ::google::protobuf::Message() {
  SharedCtor();
  // @@protoc_insertion_point(constructor:MaxBotMessage.Vector3)
}

void Vector3::InitAsDefaultInstance() {
}

Vector3::Vector3(const Vector3& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:MaxBotMessage.Vector3)
}

void Vector3::SharedCtor() {
  _cached_size_ = 0;
  x_ = 0;
  y_ = 0;
  z_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

Vector3::~Vector3() {
  // @@protoc_insertion_point(destructor:MaxBotMessage.Vector3)
  SharedDtor();
}

void Vector3::SharedDtor() {
  if (this != default_instance_) {
  }
}

void Vector3::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Vector3::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Vector3_descriptor_;
}

const Vector3& Vector3::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_Messages_2fGeometry_2fVector3_2eproto();
  return *default_instance_;
}

Vector3* Vector3::default_instance_ = NULL;

Vector3* Vector3::New() const {
  return new Vector3;
}

void Vector3::Clear() {
#define OFFSET_OF_FIELD_(f) (reinterpret_cast<char*>(      \
  &reinterpret_cast<Vector3*>(16)->f) - \
   reinterpret_cast<char*>(16))

#define ZR_(first, last) do {                              \
    size_t f = OFFSET_OF_FIELD_(first);                    \
    size_t n = OFFSET_OF_FIELD_(last) - f + sizeof(last);  \
    ::memset(&first, 0, n);                                \
  } while (0)

  ZR_(x_, z_);

#undef OFFSET_OF_FIELD_
#undef ZR_

  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool Vector3::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:MaxBotMessage.Vector3)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required double x = 1;
      case 1: {
        if (tag == 9) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &x_)));
          set_has_x();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(17)) goto parse_y;
        break;
      }

      // required double y = 2;
      case 2: {
        if (tag == 17) {
         parse_y:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &y_)));
          set_has_y();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(25)) goto parse_z;
        break;
      }

      // required double z = 3;
      case 3: {
        if (tag == 25) {
         parse_z:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &z_)));
          set_has_z();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:MaxBotMessage.Vector3)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:MaxBotMessage.Vector3)
  return false;
#undef DO_
}

void Vector3::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:MaxBotMessage.Vector3)
  // required double x = 1;
  if (has_x()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->x(), output);
  }

  // required double y = 2;
  if (has_y()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->y(), output);
  }

  // required double z = 3;
  if (has_z()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(3, this->z(), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:MaxBotMessage.Vector3)
}

::google::protobuf::uint8* Vector3::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:MaxBotMessage.Vector3)
  // required double x = 1;
  if (has_x()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->x(), target);
  }

  // required double y = 2;
  if (has_y()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->y(), target);
  }

  // required double z = 3;
  if (has_z()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(3, this->z(), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:MaxBotMessage.Vector3)
  return target;
}

int Vector3::ByteSize() const {
  int total_size = 0;

  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // required double x = 1;
    if (has_x()) {
      total_size += 1 + 8;
    }

    // required double y = 2;
    if (has_y()) {
      total_size += 1 + 8;
    }

    // required double z = 3;
    if (has_z()) {
      total_size += 1 + 8;
    }

  }
  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Vector3::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const Vector3* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const Vector3*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void Vector3::MergeFrom(const Vector3& from) {
  GOOGLE_CHECK_NE(&from, this);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_x()) {
      set_x(from.x());
    }
    if (from.has_y()) {
      set_y(from.y());
    }
    if (from.has_z()) {
      set_z(from.z());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void Vector3::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Vector3::CopyFrom(const Vector3& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Vector3::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000007) != 0x00000007) return false;

  return true;
}

void Vector3::Swap(Vector3* other) {
  if (other != this) {
    std::swap(x_, other->x_);
    std::swap(y_, other->y_);
    std::swap(z_, other->z_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata Vector3::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = Vector3_descriptor_;
  metadata.reflection = Vector3_reflection_;
  return metadata;
}


// ===================================================================

#ifndef _MSC_VER
const int Vector3Stamped::kStampFieldNumber;
const int Vector3Stamped::kVectorFieldNumber;
#endif  // !_MSC_VER

Vector3Stamped::Vector3Stamped()
  : ::google::protobuf::Message() {
  SharedCtor();
  // @@protoc_insertion_point(constructor:MaxBotMessage.Vector3Stamped)
}

void Vector3Stamped::InitAsDefaultInstance() {
  stamp_ = const_cast< ::MaxBotMessage::Stamp*>(&::MaxBotMessage::Stamp::default_instance());
  vector_ = const_cast< ::MaxBotMessage::Vector3*>(&::MaxBotMessage::Vector3::default_instance());
}

Vector3Stamped::Vector3Stamped(const Vector3Stamped& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:MaxBotMessage.Vector3Stamped)
}

void Vector3Stamped::SharedCtor() {
  _cached_size_ = 0;
  stamp_ = NULL;
  vector_ = NULL;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

Vector3Stamped::~Vector3Stamped() {
  // @@protoc_insertion_point(destructor:MaxBotMessage.Vector3Stamped)
  SharedDtor();
}

void Vector3Stamped::SharedDtor() {
  if (this != default_instance_) {
    delete stamp_;
    delete vector_;
  }
}

void Vector3Stamped::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Vector3Stamped::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Vector3Stamped_descriptor_;
}

const Vector3Stamped& Vector3Stamped::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_Messages_2fGeometry_2fVector3_2eproto();
  return *default_instance_;
}

Vector3Stamped* Vector3Stamped::default_instance_ = NULL;

Vector3Stamped* Vector3Stamped::New() const {
  return new Vector3Stamped;
}

void Vector3Stamped::Clear() {
  if (_has_bits_[0 / 32] & 3) {
    if (has_stamp()) {
      if (stamp_ != NULL) stamp_->::MaxBotMessage::Stamp::Clear();
    }
    if (has_vector()) {
      if (vector_ != NULL) vector_->::MaxBotMessage::Vector3::Clear();
    }
  }
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool Vector3Stamped::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:MaxBotMessage.Vector3Stamped)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required .MaxBotMessage.Stamp stamp = 1;
      case 1: {
        if (tag == 10) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_stamp()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(18)) goto parse_vector;
        break;
      }

      // required .MaxBotMessage.Vector3 vector = 2;
      case 2: {
        if (tag == 18) {
         parse_vector:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_vector()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:MaxBotMessage.Vector3Stamped)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:MaxBotMessage.Vector3Stamped)
  return false;
#undef DO_
}

void Vector3Stamped::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:MaxBotMessage.Vector3Stamped)
  // required .MaxBotMessage.Stamp stamp = 1;
  if (has_stamp()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->stamp(), output);
  }

  // required .MaxBotMessage.Vector3 vector = 2;
  if (has_vector()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2, this->vector(), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:MaxBotMessage.Vector3Stamped)
}

::google::protobuf::uint8* Vector3Stamped::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:MaxBotMessage.Vector3Stamped)
  // required .MaxBotMessage.Stamp stamp = 1;
  if (has_stamp()) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        1, this->stamp(), target);
  }

  // required .MaxBotMessage.Vector3 vector = 2;
  if (has_vector()) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        2, this->vector(), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:MaxBotMessage.Vector3Stamped)
  return target;
}

int Vector3Stamped::ByteSize() const {
  int total_size = 0;

  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // required .MaxBotMessage.Stamp stamp = 1;
    if (has_stamp()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          this->stamp());
    }

    // required .MaxBotMessage.Vector3 vector = 2;
    if (has_vector()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          this->vector());
    }

  }
  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Vector3Stamped::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const Vector3Stamped* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const Vector3Stamped*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void Vector3Stamped::MergeFrom(const Vector3Stamped& from) {
  GOOGLE_CHECK_NE(&from, this);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_stamp()) {
      mutable_stamp()->::MaxBotMessage::Stamp::MergeFrom(from.stamp());
    }
    if (from.has_vector()) {
      mutable_vector()->::MaxBotMessage::Vector3::MergeFrom(from.vector());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void Vector3Stamped::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Vector3Stamped::CopyFrom(const Vector3Stamped& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Vector3Stamped::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000003) != 0x00000003) return false;

  if (has_stamp()) {
    if (!this->stamp().IsInitialized()) return false;
  }
  if (has_vector()) {
    if (!this->vector().IsInitialized()) return false;
  }
  return true;
}

void Vector3Stamped::Swap(Vector3Stamped* other) {
  if (other != this) {
    std::swap(stamp_, other->stamp_);
    std::swap(vector_, other->vector_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata Vector3Stamped::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = Vector3Stamped_descriptor_;
  metadata.reflection = Vector3Stamped_reflection_;
  return metadata;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace MaxBotMessage

// @@protoc_insertion_point(global_scope)
