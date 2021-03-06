// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Imu.proto

#include "Imu.pb.h"

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
namespace protobuf_quaternion_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_quaternion_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_Quaternion;
}  // namespace protobuf_quaternion_2eproto
namespace protobuf_vector3d_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_vector3d_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_Vector3d;
}  // namespace protobuf_vector3d_2eproto
namespace gz_sensor_msgs {
class ImuDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<Imu>
      _instance;
} _Imu_default_instance_;
}  // namespace gz_sensor_msgs
namespace protobuf_Imu_2eproto {
static void InitDefaultsImu() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::gz_sensor_msgs::_Imu_default_instance_;
    new (ptr) ::gz_sensor_msgs::Imu();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::gz_sensor_msgs::Imu::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<3> scc_info_Imu =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 3, InitDefaultsImu}, {
      &protobuf_Header_2eproto::scc_info_Header.base,
      &protobuf_quaternion_2eproto::scc_info_Quaternion.base,
      &protobuf_vector3d_2eproto::scc_info_Vector3d.base,}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_Imu.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gz_sensor_msgs::Imu, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gz_sensor_msgs::Imu, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gz_sensor_msgs::Imu, header_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gz_sensor_msgs::Imu, orientation_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gz_sensor_msgs::Imu, orientation_covariance_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gz_sensor_msgs::Imu, angular_velocity_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gz_sensor_msgs::Imu, angular_velocity_covariance_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gz_sensor_msgs::Imu, linear_acceleration_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gz_sensor_msgs::Imu, linear_acceleration_covariance_),
  0,
  1,
  ~0u,
  2,
  ~0u,
  3,
  ~0u,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 12, sizeof(::gz_sensor_msgs::Imu)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::gz_sensor_msgs::_Imu_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "Imu.proto", schemas, file_default_instances, TableStruct::offsets,
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
      "\n\tImu.proto\022\016gz_sensor_msgs\032\020quaternion."
      "proto\032\016vector3d.proto\032\014Header.proto\"\266\002\n\003"
      "Imu\022#\n\006header\030\001 \002(\0132\023.gz_std_msgs.Header"
      "\022,\n\013orientation\030\002 \002(\0132\027.gazebo.msgs.Quat"
      "ernion\022\"\n\026orientation_covariance\030\003 \003(\002B\002"
      "\020\001\022/\n\020angular_velocity\030\004 \002(\0132\025.gazebo.ms"
      "gs.Vector3d\022\'\n\033angular_velocity_covarian"
      "ce\030\005 \003(\002B\002\020\001\0222\n\023linear_acceleration\030\006 \002("
      "\0132\025.gazebo.msgs.Vector3d\022*\n\036linear_accel"
      "eration_covariance\030\007 \003(\002B\002\020\001"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 388);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "Imu.proto", &protobuf_RegisterTypes);
  ::protobuf_quaternion_2eproto::AddDescriptors();
  ::protobuf_vector3d_2eproto::AddDescriptors();
  ::protobuf_Header_2eproto::AddDescriptors();
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
}  // namespace protobuf_Imu_2eproto
namespace gz_sensor_msgs {

// ===================================================================

void Imu::InitAsDefaultInstance() {
  ::gz_sensor_msgs::_Imu_default_instance_._instance.get_mutable()->header_ = const_cast< ::gz_std_msgs::Header*>(
      ::gz_std_msgs::Header::internal_default_instance());
  ::gz_sensor_msgs::_Imu_default_instance_._instance.get_mutable()->orientation_ = const_cast< ::gazebo::msgs::Quaternion*>(
      ::gazebo::msgs::Quaternion::internal_default_instance());
  ::gz_sensor_msgs::_Imu_default_instance_._instance.get_mutable()->angular_velocity_ = const_cast< ::gazebo::msgs::Vector3d*>(
      ::gazebo::msgs::Vector3d::internal_default_instance());
  ::gz_sensor_msgs::_Imu_default_instance_._instance.get_mutable()->linear_acceleration_ = const_cast< ::gazebo::msgs::Vector3d*>(
      ::gazebo::msgs::Vector3d::internal_default_instance());
}
void Imu::clear_header() {
  if (header_ != NULL) header_->Clear();
  clear_has_header();
}
void Imu::clear_orientation() {
  if (orientation_ != NULL) orientation_->Clear();
  clear_has_orientation();
}
void Imu::clear_angular_velocity() {
  if (angular_velocity_ != NULL) angular_velocity_->Clear();
  clear_has_angular_velocity();
}
void Imu::clear_linear_acceleration() {
  if (linear_acceleration_ != NULL) linear_acceleration_->Clear();
  clear_has_linear_acceleration();
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Imu::kHeaderFieldNumber;
const int Imu::kOrientationFieldNumber;
const int Imu::kOrientationCovarianceFieldNumber;
const int Imu::kAngularVelocityFieldNumber;
const int Imu::kAngularVelocityCovarianceFieldNumber;
const int Imu::kLinearAccelerationFieldNumber;
const int Imu::kLinearAccelerationCovarianceFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Imu::Imu()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_Imu_2eproto::scc_info_Imu.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:gz_sensor_msgs.Imu)
}
Imu::Imu(const Imu& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      orientation_covariance_(from.orientation_covariance_),
      angular_velocity_covariance_(from.angular_velocity_covariance_),
      linear_acceleration_covariance_(from.linear_acceleration_covariance_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_header()) {
    header_ = new ::gz_std_msgs::Header(*from.header_);
  } else {
    header_ = NULL;
  }
  if (from.has_orientation()) {
    orientation_ = new ::gazebo::msgs::Quaternion(*from.orientation_);
  } else {
    orientation_ = NULL;
  }
  if (from.has_angular_velocity()) {
    angular_velocity_ = new ::gazebo::msgs::Vector3d(*from.angular_velocity_);
  } else {
    angular_velocity_ = NULL;
  }
  if (from.has_linear_acceleration()) {
    linear_acceleration_ = new ::gazebo::msgs::Vector3d(*from.linear_acceleration_);
  } else {
    linear_acceleration_ = NULL;
  }
  // @@protoc_insertion_point(copy_constructor:gz_sensor_msgs.Imu)
}

void Imu::SharedCtor() {
  ::memset(&header_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&linear_acceleration_) -
      reinterpret_cast<char*>(&header_)) + sizeof(linear_acceleration_));
}

Imu::~Imu() {
  // @@protoc_insertion_point(destructor:gz_sensor_msgs.Imu)
  SharedDtor();
}

void Imu::SharedDtor() {
  if (this != internal_default_instance()) delete header_;
  if (this != internal_default_instance()) delete orientation_;
  if (this != internal_default_instance()) delete angular_velocity_;
  if (this != internal_default_instance()) delete linear_acceleration_;
}

void Imu::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* Imu::descriptor() {
  ::protobuf_Imu_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_Imu_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const Imu& Imu::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_Imu_2eproto::scc_info_Imu.base);
  return *internal_default_instance();
}


void Imu::Clear() {
// @@protoc_insertion_point(message_clear_start:gz_sensor_msgs.Imu)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  orientation_covariance_.Clear();
  angular_velocity_covariance_.Clear();
  linear_acceleration_covariance_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 15u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(header_ != NULL);
      header_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(orientation_ != NULL);
      orientation_->Clear();
    }
    if (cached_has_bits & 0x00000004u) {
      GOOGLE_DCHECK(angular_velocity_ != NULL);
      angular_velocity_->Clear();
    }
    if (cached_has_bits & 0x00000008u) {
      GOOGLE_DCHECK(linear_acceleration_ != NULL);
      linear_acceleration_->Clear();
    }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool Imu::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:gz_sensor_msgs.Imu)
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

      // required .gazebo.msgs.Quaternion orientation = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_orientation()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated float orientation_covariance = 3 [packed = true];
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(26u /* 26 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, this->mutable_orientation_covariance())));
        } else if (
            static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(29u /* 29 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 1, 26u, input, this->mutable_orientation_covariance())));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required .gazebo.msgs.Vector3d angular_velocity = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(34u /* 34 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_angular_velocity()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated float angular_velocity_covariance = 5 [packed = true];
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(42u /* 42 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, this->mutable_angular_velocity_covariance())));
        } else if (
            static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(45u /* 45 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 1, 42u, input, this->mutable_angular_velocity_covariance())));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required .gazebo.msgs.Vector3d linear_acceleration = 6;
      case 6: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(50u /* 50 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_linear_acceleration()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated float linear_acceleration_covariance = 7 [packed = true];
      case 7: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(58u /* 58 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, this->mutable_linear_acceleration_covariance())));
        } else if (
            static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(61u /* 61 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 1, 58u, input, this->mutable_linear_acceleration_covariance())));
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
  // @@protoc_insertion_point(parse_success:gz_sensor_msgs.Imu)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:gz_sensor_msgs.Imu)
  return false;
#undef DO_
}

void Imu::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:gz_sensor_msgs.Imu)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required .gz_std_msgs.Header header = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->_internal_header(), output);
  }

  // required .gazebo.msgs.Quaternion orientation = 2;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2, this->_internal_orientation(), output);
  }

  // repeated float orientation_covariance = 3 [packed = true];
  if (this->orientation_covariance_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(3, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(static_cast< ::google::protobuf::uint32>(
        _orientation_covariance_cached_byte_size_));
    ::google::protobuf::internal::WireFormatLite::WriteFloatArray(
      this->orientation_covariance().data(), this->orientation_covariance_size(), output);
  }

  // required .gazebo.msgs.Vector3d angular_velocity = 4;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      4, this->_internal_angular_velocity(), output);
  }

  // repeated float angular_velocity_covariance = 5 [packed = true];
  if (this->angular_velocity_covariance_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(5, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(static_cast< ::google::protobuf::uint32>(
        _angular_velocity_covariance_cached_byte_size_));
    ::google::protobuf::internal::WireFormatLite::WriteFloatArray(
      this->angular_velocity_covariance().data(), this->angular_velocity_covariance_size(), output);
  }

  // required .gazebo.msgs.Vector3d linear_acceleration = 6;
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      6, this->_internal_linear_acceleration(), output);
  }

  // repeated float linear_acceleration_covariance = 7 [packed = true];
  if (this->linear_acceleration_covariance_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(7, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(static_cast< ::google::protobuf::uint32>(
        _linear_acceleration_covariance_cached_byte_size_));
    ::google::protobuf::internal::WireFormatLite::WriteFloatArray(
      this->linear_acceleration_covariance().data(), this->linear_acceleration_covariance_size(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:gz_sensor_msgs.Imu)
}

::google::protobuf::uint8* Imu::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:gz_sensor_msgs.Imu)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required .gz_std_msgs.Header header = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, this->_internal_header(), deterministic, target);
  }

  // required .gazebo.msgs.Quaternion orientation = 2;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        2, this->_internal_orientation(), deterministic, target);
  }

  // repeated float orientation_covariance = 3 [packed = true];
  if (this->orientation_covariance_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      3,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
        static_cast< ::google::protobuf::int32>(
            _orientation_covariance_cached_byte_size_), target);
    target = ::google::protobuf::internal::WireFormatLite::
      WriteFloatNoTagToArray(this->orientation_covariance_, target);
  }

  // required .gazebo.msgs.Vector3d angular_velocity = 4;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        4, this->_internal_angular_velocity(), deterministic, target);
  }

  // repeated float angular_velocity_covariance = 5 [packed = true];
  if (this->angular_velocity_covariance_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      5,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
        static_cast< ::google::protobuf::int32>(
            _angular_velocity_covariance_cached_byte_size_), target);
    target = ::google::protobuf::internal::WireFormatLite::
      WriteFloatNoTagToArray(this->angular_velocity_covariance_, target);
  }

  // required .gazebo.msgs.Vector3d linear_acceleration = 6;
  if (cached_has_bits & 0x00000008u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        6, this->_internal_linear_acceleration(), deterministic, target);
  }

  // repeated float linear_acceleration_covariance = 7 [packed = true];
  if (this->linear_acceleration_covariance_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      7,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
        static_cast< ::google::protobuf::int32>(
            _linear_acceleration_covariance_cached_byte_size_), target);
    target = ::google::protobuf::internal::WireFormatLite::
      WriteFloatNoTagToArray(this->linear_acceleration_covariance_, target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:gz_sensor_msgs.Imu)
  return target;
}

size_t Imu::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:gz_sensor_msgs.Imu)
  size_t total_size = 0;

  if (has_header()) {
    // required .gz_std_msgs.Header header = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *header_);
  }

  if (has_orientation()) {
    // required .gazebo.msgs.Quaternion orientation = 2;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *orientation_);
  }

  if (has_angular_velocity()) {
    // required .gazebo.msgs.Vector3d angular_velocity = 4;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *angular_velocity_);
  }

  if (has_linear_acceleration()) {
    // required .gazebo.msgs.Vector3d linear_acceleration = 6;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *linear_acceleration_);
  }

  return total_size;
}
size_t Imu::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:gz_sensor_msgs.Imu)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (((_has_bits_[0] & 0x0000000f) ^ 0x0000000f) == 0) {  // All required fields are present.
    // required .gz_std_msgs.Header header = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *header_);

    // required .gazebo.msgs.Quaternion orientation = 2;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *orientation_);

    // required .gazebo.msgs.Vector3d angular_velocity = 4;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *angular_velocity_);

    // required .gazebo.msgs.Vector3d linear_acceleration = 6;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *linear_acceleration_);

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  // repeated float orientation_covariance = 3 [packed = true];
  {
    unsigned int count = static_cast<unsigned int>(this->orientation_covariance_size());
    size_t data_size = 4UL * count;
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
            static_cast< ::google::protobuf::int32>(data_size));
    }
    int cached_size = ::google::protobuf::internal::ToCachedSize(data_size);
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _orientation_covariance_cached_byte_size_ = cached_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  // repeated float angular_velocity_covariance = 5 [packed = true];
  {
    unsigned int count = static_cast<unsigned int>(this->angular_velocity_covariance_size());
    size_t data_size = 4UL * count;
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
            static_cast< ::google::protobuf::int32>(data_size));
    }
    int cached_size = ::google::protobuf::internal::ToCachedSize(data_size);
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _angular_velocity_covariance_cached_byte_size_ = cached_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  // repeated float linear_acceleration_covariance = 7 [packed = true];
  {
    unsigned int count = static_cast<unsigned int>(this->linear_acceleration_covariance_size());
    size_t data_size = 4UL * count;
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
            static_cast< ::google::protobuf::int32>(data_size));
    }
    int cached_size = ::google::protobuf::internal::ToCachedSize(data_size);
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _linear_acceleration_covariance_cached_byte_size_ = cached_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Imu::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:gz_sensor_msgs.Imu)
  GOOGLE_DCHECK_NE(&from, this);
  const Imu* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const Imu>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:gz_sensor_msgs.Imu)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:gz_sensor_msgs.Imu)
    MergeFrom(*source);
  }
}

void Imu::MergeFrom(const Imu& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:gz_sensor_msgs.Imu)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  orientation_covariance_.MergeFrom(from.orientation_covariance_);
  angular_velocity_covariance_.MergeFrom(from.angular_velocity_covariance_);
  linear_acceleration_covariance_.MergeFrom(from.linear_acceleration_covariance_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 15u) {
    if (cached_has_bits & 0x00000001u) {
      mutable_header()->::gz_std_msgs::Header::MergeFrom(from.header());
    }
    if (cached_has_bits & 0x00000002u) {
      mutable_orientation()->::gazebo::msgs::Quaternion::MergeFrom(from.orientation());
    }
    if (cached_has_bits & 0x00000004u) {
      mutable_angular_velocity()->::gazebo::msgs::Vector3d::MergeFrom(from.angular_velocity());
    }
    if (cached_has_bits & 0x00000008u) {
      mutable_linear_acceleration()->::gazebo::msgs::Vector3d::MergeFrom(from.linear_acceleration());
    }
  }
}

void Imu::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:gz_sensor_msgs.Imu)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Imu::CopyFrom(const Imu& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:gz_sensor_msgs.Imu)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Imu::IsInitialized() const {
  if ((_has_bits_[0] & 0x0000000f) != 0x0000000f) return false;
  if (has_header()) {
    if (!this->header_->IsInitialized()) return false;
  }
  if (has_orientation()) {
    if (!this->orientation_->IsInitialized()) return false;
  }
  if (has_angular_velocity()) {
    if (!this->angular_velocity_->IsInitialized()) return false;
  }
  if (has_linear_acceleration()) {
    if (!this->linear_acceleration_->IsInitialized()) return false;
  }
  return true;
}

void Imu::Swap(Imu* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Imu::InternalSwap(Imu* other) {
  using std::swap;
  orientation_covariance_.InternalSwap(&other->orientation_covariance_);
  angular_velocity_covariance_.InternalSwap(&other->angular_velocity_covariance_);
  linear_acceleration_covariance_.InternalSwap(&other->linear_acceleration_covariance_);
  swap(header_, other->header_);
  swap(orientation_, other->orientation_);
  swap(angular_velocity_, other->angular_velocity_);
  swap(linear_acceleration_, other->linear_acceleration_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata Imu::GetMetadata() const {
  protobuf_Imu_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_Imu_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace gz_sensor_msgs
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::gz_sensor_msgs::Imu* Arena::CreateMaybeMessage< ::gz_sensor_msgs::Imu >(Arena* arena) {
  return Arena::CreateInternal< ::gz_sensor_msgs::Imu >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
