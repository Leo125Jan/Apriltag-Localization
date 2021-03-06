// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: RollPitchYawrateThrust.proto

#include "RollPitchYawrateThrust.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)

namespace protobuf_Header_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_Header_2eproto ::google::protobuf::internal::SCCInfo<1> scc_info_Header;
}  // namespace protobuf_Header_2eproto
namespace protobuf_vector3d_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_vector3d_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_Vector3d;
}  // namespace protobuf_vector3d_2eproto
namespace gz_mav_msgs {
class RollPitchYawrateThrustDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<RollPitchYawrateThrust>
      _instance;
} _RollPitchYawrateThrust_default_instance_;
}  // namespace gz_mav_msgs
namespace protobuf_RollPitchYawrateThrust_2eproto {
static void InitDefaultsRollPitchYawrateThrust() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::gz_mav_msgs::_RollPitchYawrateThrust_default_instance_;
    new (ptr) ::gz_mav_msgs::RollPitchYawrateThrust();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::gz_mav_msgs::RollPitchYawrateThrust::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<2> scc_info_RollPitchYawrateThrust =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 2, InitDefaultsRollPitchYawrateThrust}, {
      &protobuf_Header_2eproto::scc_info_Header.base,
      &protobuf_vector3d_2eproto::scc_info_Vector3d.base,}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_RollPitchYawrateThrust.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gz_mav_msgs::RollPitchYawrateThrust, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gz_mav_msgs::RollPitchYawrateThrust, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gz_mav_msgs::RollPitchYawrateThrust, header_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gz_mav_msgs::RollPitchYawrateThrust, roll_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gz_mav_msgs::RollPitchYawrateThrust, pitch_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gz_mav_msgs::RollPitchYawrateThrust, yaw_rate_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gz_mav_msgs::RollPitchYawrateThrust, thrust_),
  0,
  2,
  3,
  4,
  1,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 10, sizeof(::gz_mav_msgs::RollPitchYawrateThrust)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::gz_mav_msgs::_RollPitchYawrateThrust_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "RollPitchYawrateThrust.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\034RollPitchYawrateThrust.proto\022\013gz_mav_m"
      "sgs\032\014Header.proto\032\016vector3d.proto\"\223\001\n\026Ro"
      "llPitchYawrateThrust\022#\n\006header\030\001 \002(\0132\023.g"
      "z_std_msgs.Header\022\014\n\004roll\030\002 \002(\001\022\r\n\005pitch"
      "\030\003 \002(\001\022\020\n\010yaw_rate\030\004 \002(\001\022%\n\006thrust\030\005 \002(\013"
      "2\025.gazebo.msgs.Vector3d"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 223);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "RollPitchYawrateThrust.proto", &protobuf_RegisterTypes);
  ::protobuf_Header_2eproto::AddDescriptors();
  ::protobuf_vector3d_2eproto::AddDescriptors();
}

void AddDescriptors() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_RollPitchYawrateThrust_2eproto
namespace gz_mav_msgs {

// ===================================================================

void RollPitchYawrateThrust::InitAsDefaultInstance() {
  ::gz_mav_msgs::_RollPitchYawrateThrust_default_instance_._instance.get_mutable()->header_ = const_cast< ::gz_std_msgs::Header*>(
      ::gz_std_msgs::Header::internal_default_instance());
  ::gz_mav_msgs::_RollPitchYawrateThrust_default_instance_._instance.get_mutable()->thrust_ = const_cast< ::gazebo::msgs::Vector3d*>(
      ::gazebo::msgs::Vector3d::internal_default_instance());
}
void RollPitchYawrateThrust::clear_header() {
  if (header_ != NULL) header_->Clear();
  clear_has_header();
}
void RollPitchYawrateThrust::clear_thrust() {
  if (thrust_ != NULL) thrust_->Clear();
  clear_has_thrust();
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int RollPitchYawrateThrust::kHeaderFieldNumber;
const int RollPitchYawrateThrust::kRollFieldNumber;
const int RollPitchYawrateThrust::kPitchFieldNumber;
const int RollPitchYawrateThrust::kYawRateFieldNumber;
const int RollPitchYawrateThrust::kThrustFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

RollPitchYawrateThrust::RollPitchYawrateThrust()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_RollPitchYawrateThrust_2eproto::scc_info_RollPitchYawrateThrust.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:gz_mav_msgs.RollPitchYawrateThrust)
}
RollPitchYawrateThrust::RollPitchYawrateThrust(const RollPitchYawrateThrust& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_header()) {
    header_ = new ::gz_std_msgs::Header(*from.header_);
  } else {
    header_ = NULL;
  }
  if (from.has_thrust()) {
    thrust_ = new ::gazebo::msgs::Vector3d(*from.thrust_);
  } else {
    thrust_ = NULL;
  }
  ::memcpy(&roll_, &from.roll_,
    static_cast<size_t>(reinterpret_cast<char*>(&yaw_rate_) -
    reinterpret_cast<char*>(&roll_)) + sizeof(yaw_rate_));
  // @@protoc_insertion_point(copy_constructor:gz_mav_msgs.RollPitchYawrateThrust)
}

void RollPitchYawrateThrust::SharedCtor() {
  ::memset(&header_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&yaw_rate_) -
      reinterpret_cast<char*>(&header_)) + sizeof(yaw_rate_));
}

RollPitchYawrateThrust::~RollPitchYawrateThrust() {
  // @@protoc_insertion_point(destructor:gz_mav_msgs.RollPitchYawrateThrust)
  SharedDtor();
}

void RollPitchYawrateThrust::SharedDtor() {
  if (this != internal_default_instance()) delete header_;
  if (this != internal_default_instance()) delete thrust_;
}

void RollPitchYawrateThrust::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* RollPitchYawrateThrust::descriptor() {
  ::protobuf_RollPitchYawrateThrust_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_RollPitchYawrateThrust_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const RollPitchYawrateThrust& RollPitchYawrateThrust::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_RollPitchYawrateThrust_2eproto::scc_info_RollPitchYawrateThrust.base);
  return *internal_default_instance();
}


void RollPitchYawrateThrust::Clear() {
// @@protoc_insertion_point(message_clear_start:gz_mav_msgs.RollPitchYawrateThrust)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 3u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(header_ != NULL);
      header_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(thrust_ != NULL);
      thrust_->Clear();
    }
  }
  if (cached_has_bits & 28u) {
    ::memset(&roll_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&yaw_rate_) -
        reinterpret_cast<char*>(&roll_)) + sizeof(yaw_rate_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool RollPitchYawrateThrust::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:gz_mav_msgs.RollPitchYawrateThrust)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required .gz_std_msgs.Header header = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_header()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required double roll = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(17u /* 17 & 0xFF */)) {
          set_has_roll();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &roll_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required double pitch = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(25u /* 25 & 0xFF */)) {
          set_has_pitch();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &pitch_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required double yaw_rate = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(33u /* 33 & 0xFF */)) {
          set_has_yaw_rate();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &yaw_rate_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required .gazebo.msgs.Vector3d thrust = 5;
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(42u /* 42 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_thrust()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:gz_mav_msgs.RollPitchYawrateThrust)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:gz_mav_msgs.RollPitchYawrateThrust)
  return false;
#undef DO_
}

void RollPitchYawrateThrust::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:gz_mav_msgs.RollPitchYawrateThrust)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required .gz_std_msgs.Header header = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->_internal_header(), output);
  }

  // required double roll = 2;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->roll(), output);
  }

  // required double pitch = 3;
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(3, this->pitch(), output);
  }

  // required double yaw_rate = 4;
  if (cached_has_bits & 0x00000010u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(4, this->yaw_rate(), output);
  }

  // required .gazebo.msgs.Vector3d thrust = 5;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      5, this->_internal_thrust(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:gz_mav_msgs.RollPitchYawrateThrust)
}

::google::protobuf::uint8* RollPitchYawrateThrust::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:gz_mav_msgs.RollPitchYawrateThrust)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required .gz_std_msgs.Header header = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, this->_internal_header(), deterministic, target);
  }

  // required double roll = 2;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->roll(), target);
  }

  // required double pitch = 3;
  if (cached_has_bits & 0x00000008u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(3, this->pitch(), target);
  }

  // required double yaw_rate = 4;
  if (cached_has_bits & 0x00000010u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(4, this->yaw_rate(), target);
  }

  // required .gazebo.msgs.Vector3d thrust = 5;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        5, this->_internal_thrust(), deterministic, target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:gz_mav_msgs.RollPitchYawrateThrust)
  return target;
}

size_t RollPitchYawrateThrust::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:gz_mav_msgs.RollPitchYawrateThrust)
  size_t total_size = 0;

  if (has_header()) {
    // required .gz_std_msgs.Header header = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *header_);
  }

  if (has_thrust()) {
    // required .gazebo.msgs.Vector3d thrust = 5;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *thrust_);
  }

  if (has_roll()) {
    // required double roll = 2;
    total_size += 1 + 8;
  }

  if (has_pitch()) {
    // required double pitch = 3;
    total_size += 1 + 8;
  }

  if (has_yaw_rate()) {
    // required double yaw_rate = 4;
    total_size += 1 + 8;
  }

  return total_size;
}
size_t RollPitchYawrateThrust::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:gz_mav_msgs.RollPitchYawrateThrust)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (((_has_bits_[0] & 0x0000001f) ^ 0x0000001f) == 0) {  // All required fields are present.
    // required .gz_std_msgs.Header header = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *header_);

    // required .gazebo.msgs.Vector3d thrust = 5;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *thrust_);

    // required double roll = 2;
    total_size += 1 + 8;

    // required double pitch = 3;
    total_size += 1 + 8;

    // required double yaw_rate = 4;
    total_size += 1 + 8;

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void RollPitchYawrateThrust::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:gz_mav_msgs.RollPitchYawrateThrust)
  GOOGLE_DCHECK_NE(&from, this);
  const RollPitchYawrateThrust* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const RollPitchYawrateThrust>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:gz_mav_msgs.RollPitchYawrateThrust)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:gz_mav_msgs.RollPitchYawrateThrust)
    MergeFrom(*source);
  }
}

void RollPitchYawrateThrust::MergeFrom(const RollPitchYawrateThrust& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:gz_mav_msgs.RollPitchYawrateThrust)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 31u) {
    if (cached_has_bits & 0x00000001u) {
      mutable_header()->::gz_std_msgs::Header::MergeFrom(from.header());
    }
    if (cached_has_bits & 0x00000002u) {
      mutable_thrust()->::gazebo::msgs::Vector3d::MergeFrom(from.thrust());
    }
    if (cached_has_bits & 0x00000004u) {
      roll_ = from.roll_;
    }
    if (cached_has_bits & 0x00000008u) {
      pitch_ = from.pitch_;
    }
    if (cached_has_bits & 0x00000010u) {
      yaw_rate_ = from.yaw_rate_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void RollPitchYawrateThrust::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:gz_mav_msgs.RollPitchYawrateThrust)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void RollPitchYawrateThrust::CopyFrom(const RollPitchYawrateThrust& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:gz_mav_msgs.RollPitchYawrateThrust)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool RollPitchYawrateThrust::IsInitialized() const {
  if ((_has_bits_[0] & 0x0000001f) != 0x0000001f) return false;
  if (has_header()) {
    if (!this->header_->IsInitialized()) return false;
  }
  if (has_thrust()) {
    if (!this->thrust_->IsInitialized()) return false;
  }
  return true;
}

void RollPitchYawrateThrust::Swap(RollPitchYawrateThrust* other) {
  if (other == this) return;
  InternalSwap(other);
}
void RollPitchYawrateThrust::InternalSwap(RollPitchYawrateThrust* other) {
  using std::swap;
  swap(header_, other->header_);
  swap(thrust_, other->thrust_);
  swap(roll_, other->roll_);
  swap(pitch_, other->pitch_);
  swap(yaw_rate_, other->yaw_rate_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata RollPitchYawrateThrust::GetMetadata() const {
  protobuf_RollPitchYawrateThrust_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_RollPitchYawrateThrust_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace gz_mav_msgs
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::gz_mav_msgs::RollPitchYawrateThrust* Arena::CreateMaybeMessage< ::gz_mav_msgs::RollPitchYawrateThrust >(Arena* arena) {
  return Arena::CreateInternal< ::gz_mav_msgs::RollPitchYawrateThrust >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
