// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: OpticalFlow.proto

#include "OpticalFlow.pb.h"

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

namespace opticalFlow_msgs {
namespace msgs {
class opticalFlowDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<opticalFlow>
      _instance;
} _opticalFlow_default_instance_;
}  // namespace msgs
}  // namespace opticalFlow_msgs
namespace protobuf_OpticalFlow_2eproto {
static void InitDefaultsopticalFlow() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::opticalFlow_msgs::msgs::_opticalFlow_default_instance_;
    new (ptr) ::opticalFlow_msgs::msgs::opticalFlow();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::opticalFlow_msgs::msgs::opticalFlow::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_opticalFlow =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsopticalFlow}, {}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_opticalFlow.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::opticalFlow_msgs::msgs::opticalFlow, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::opticalFlow_msgs::msgs::opticalFlow, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::opticalFlow_msgs::msgs::opticalFlow, time_usec_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::opticalFlow_msgs::msgs::opticalFlow, sensor_id_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::opticalFlow_msgs::msgs::opticalFlow, integration_time_us_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::opticalFlow_msgs::msgs::opticalFlow, integrated_x_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::opticalFlow_msgs::msgs::opticalFlow, integrated_y_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::opticalFlow_msgs::msgs::opticalFlow, integrated_xgyro_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::opticalFlow_msgs::msgs::opticalFlow, integrated_ygyro_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::opticalFlow_msgs::msgs::opticalFlow, integrated_zgyro_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::opticalFlow_msgs::msgs::opticalFlow, temperature_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::opticalFlow_msgs::msgs::opticalFlow, quality_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::opticalFlow_msgs::msgs::opticalFlow, time_delta_distance_us_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::opticalFlow_msgs::msgs::opticalFlow, distance_),
  0,
  1,
  2,
  3,
  4,
  5,
  6,
  7,
  8,
  9,
  10,
  11,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 17, sizeof(::opticalFlow_msgs::msgs::opticalFlow)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::opticalFlow_msgs::msgs::_opticalFlow_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "OpticalFlow.proto", schemas, file_default_instances, TableStruct::offsets,
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
      "\n\021OpticalFlow.proto\022\025opticalFlow_msgs.ms"
      "gs\"\242\002\n\013opticalFlow\022\021\n\ttime_usec\030\001 \002(\002\022\021\n"
      "\tsensor_id\030\002 \002(\002\022\033\n\023integration_time_us\030"
      "\003 \002(\002\022\024\n\014integrated_x\030\004 \002(\002\022\024\n\014integrate"
      "d_y\030\005 \002(\002\022\030\n\020integrated_xgyro\030\006 \002(\002\022\030\n\020i"
      "ntegrated_ygyro\030\007 \002(\002\022\030\n\020integrated_zgyr"
      "o\030\010 \002(\002\022\023\n\013temperature\030\t \002(\002\022\017\n\007quality\030"
      "\n \002(\002\022\036\n\026time_delta_distance_us\030\013 \002(\002\022\020\n"
      "\010distance\030\014 \002(\002"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 335);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "OpticalFlow.proto", &protobuf_RegisterTypes);
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
}  // namespace protobuf_OpticalFlow_2eproto
namespace opticalFlow_msgs {
namespace msgs {

// ===================================================================

void opticalFlow::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int opticalFlow::kTimeUsecFieldNumber;
const int opticalFlow::kSensorIdFieldNumber;
const int opticalFlow::kIntegrationTimeUsFieldNumber;
const int opticalFlow::kIntegratedXFieldNumber;
const int opticalFlow::kIntegratedYFieldNumber;
const int opticalFlow::kIntegratedXgyroFieldNumber;
const int opticalFlow::kIntegratedYgyroFieldNumber;
const int opticalFlow::kIntegratedZgyroFieldNumber;
const int opticalFlow::kTemperatureFieldNumber;
const int opticalFlow::kQualityFieldNumber;
const int opticalFlow::kTimeDeltaDistanceUsFieldNumber;
const int opticalFlow::kDistanceFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

opticalFlow::opticalFlow()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_OpticalFlow_2eproto::scc_info_opticalFlow.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:opticalFlow_msgs.msgs.opticalFlow)
}
opticalFlow::opticalFlow(const opticalFlow& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&time_usec_, &from.time_usec_,
    static_cast<size_t>(reinterpret_cast<char*>(&distance_) -
    reinterpret_cast<char*>(&time_usec_)) + sizeof(distance_));
  // @@protoc_insertion_point(copy_constructor:opticalFlow_msgs.msgs.opticalFlow)
}

void opticalFlow::SharedCtor() {
  ::memset(&time_usec_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&distance_) -
      reinterpret_cast<char*>(&time_usec_)) + sizeof(distance_));
}

opticalFlow::~opticalFlow() {
  // @@protoc_insertion_point(destructor:opticalFlow_msgs.msgs.opticalFlow)
  SharedDtor();
}

void opticalFlow::SharedDtor() {
}

void opticalFlow::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* opticalFlow::descriptor() {
  ::protobuf_OpticalFlow_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_OpticalFlow_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const opticalFlow& opticalFlow::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_OpticalFlow_2eproto::scc_info_opticalFlow.base);
  return *internal_default_instance();
}


void opticalFlow::Clear() {
// @@protoc_insertion_point(message_clear_start:opticalFlow_msgs.msgs.opticalFlow)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 255u) {
    ::memset(&time_usec_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&integrated_zgyro_) -
        reinterpret_cast<char*>(&time_usec_)) + sizeof(integrated_zgyro_));
  }
  if (cached_has_bits & 3840u) {
    ::memset(&temperature_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&distance_) -
        reinterpret_cast<char*>(&temperature_)) + sizeof(distance_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool opticalFlow::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:opticalFlow_msgs.msgs.opticalFlow)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required float time_usec = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(13u /* 13 & 0xFF */)) {
          set_has_time_usec();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &time_usec_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required float sensor_id = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(21u /* 21 & 0xFF */)) {
          set_has_sensor_id();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &sensor_id_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required float integration_time_us = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(29u /* 29 & 0xFF */)) {
          set_has_integration_time_us();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &integration_time_us_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required float integrated_x = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(37u /* 37 & 0xFF */)) {
          set_has_integrated_x();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &integrated_x_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required float integrated_y = 5;
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(45u /* 45 & 0xFF */)) {
          set_has_integrated_y();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &integrated_y_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required float integrated_xgyro = 6;
      case 6: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(53u /* 53 & 0xFF */)) {
          set_has_integrated_xgyro();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &integrated_xgyro_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required float integrated_ygyro = 7;
      case 7: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(61u /* 61 & 0xFF */)) {
          set_has_integrated_ygyro();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &integrated_ygyro_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required float integrated_zgyro = 8;
      case 8: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(69u /* 69 & 0xFF */)) {
          set_has_integrated_zgyro();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &integrated_zgyro_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required float temperature = 9;
      case 9: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(77u /* 77 & 0xFF */)) {
          set_has_temperature();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &temperature_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required float quality = 10;
      case 10: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(85u /* 85 & 0xFF */)) {
          set_has_quality();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &quality_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required float time_delta_distance_us = 11;
      case 11: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(93u /* 93 & 0xFF */)) {
          set_has_time_delta_distance_us();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &time_delta_distance_us_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required float distance = 12;
      case 12: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(101u /* 101 & 0xFF */)) {
          set_has_distance();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &distance_)));
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
  // @@protoc_insertion_point(parse_success:opticalFlow_msgs.msgs.opticalFlow)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:opticalFlow_msgs.msgs.opticalFlow)
  return false;
#undef DO_
}

void opticalFlow::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:opticalFlow_msgs.msgs.opticalFlow)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required float time_usec = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(1, this->time_usec(), output);
  }

  // required float sensor_id = 2;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(2, this->sensor_id(), output);
  }

  // required float integration_time_us = 3;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(3, this->integration_time_us(), output);
  }

  // required float integrated_x = 4;
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(4, this->integrated_x(), output);
  }

  // required float integrated_y = 5;
  if (cached_has_bits & 0x00000010u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(5, this->integrated_y(), output);
  }

  // required float integrated_xgyro = 6;
  if (cached_has_bits & 0x00000020u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(6, this->integrated_xgyro(), output);
  }

  // required float integrated_ygyro = 7;
  if (cached_has_bits & 0x00000040u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(7, this->integrated_ygyro(), output);
  }

  // required float integrated_zgyro = 8;
  if (cached_has_bits & 0x00000080u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(8, this->integrated_zgyro(), output);
  }

  // required float temperature = 9;
  if (cached_has_bits & 0x00000100u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(9, this->temperature(), output);
  }

  // required float quality = 10;
  if (cached_has_bits & 0x00000200u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(10, this->quality(), output);
  }

  // required float time_delta_distance_us = 11;
  if (cached_has_bits & 0x00000400u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(11, this->time_delta_distance_us(), output);
  }

  // required float distance = 12;
  if (cached_has_bits & 0x00000800u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(12, this->distance(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:opticalFlow_msgs.msgs.opticalFlow)
}

::google::protobuf::uint8* opticalFlow::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:opticalFlow_msgs.msgs.opticalFlow)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required float time_usec = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(1, this->time_usec(), target);
  }

  // required float sensor_id = 2;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(2, this->sensor_id(), target);
  }

  // required float integration_time_us = 3;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(3, this->integration_time_us(), target);
  }

  // required float integrated_x = 4;
  if (cached_has_bits & 0x00000008u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(4, this->integrated_x(), target);
  }

  // required float integrated_y = 5;
  if (cached_has_bits & 0x00000010u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(5, this->integrated_y(), target);
  }

  // required float integrated_xgyro = 6;
  if (cached_has_bits & 0x00000020u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(6, this->integrated_xgyro(), target);
  }

  // required float integrated_ygyro = 7;
  if (cached_has_bits & 0x00000040u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(7, this->integrated_ygyro(), target);
  }

  // required float integrated_zgyro = 8;
  if (cached_has_bits & 0x00000080u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(8, this->integrated_zgyro(), target);
  }

  // required float temperature = 9;
  if (cached_has_bits & 0x00000100u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(9, this->temperature(), target);
  }

  // required float quality = 10;
  if (cached_has_bits & 0x00000200u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(10, this->quality(), target);
  }

  // required float time_delta_distance_us = 11;
  if (cached_has_bits & 0x00000400u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(11, this->time_delta_distance_us(), target);
  }

  // required float distance = 12;
  if (cached_has_bits & 0x00000800u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(12, this->distance(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:opticalFlow_msgs.msgs.opticalFlow)
  return target;
}

size_t opticalFlow::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:opticalFlow_msgs.msgs.opticalFlow)
  size_t total_size = 0;

  if (has_time_usec()) {
    // required float time_usec = 1;
    total_size += 1 + 4;
  }

  if (has_sensor_id()) {
    // required float sensor_id = 2;
    total_size += 1 + 4;
  }

  if (has_integration_time_us()) {
    // required float integration_time_us = 3;
    total_size += 1 + 4;
  }

  if (has_integrated_x()) {
    // required float integrated_x = 4;
    total_size += 1 + 4;
  }

  if (has_integrated_y()) {
    // required float integrated_y = 5;
    total_size += 1 + 4;
  }

  if (has_integrated_xgyro()) {
    // required float integrated_xgyro = 6;
    total_size += 1 + 4;
  }

  if (has_integrated_ygyro()) {
    // required float integrated_ygyro = 7;
    total_size += 1 + 4;
  }

  if (has_integrated_zgyro()) {
    // required float integrated_zgyro = 8;
    total_size += 1 + 4;
  }

  if (has_temperature()) {
    // required float temperature = 9;
    total_size += 1 + 4;
  }

  if (has_quality()) {
    // required float quality = 10;
    total_size += 1 + 4;
  }

  if (has_time_delta_distance_us()) {
    // required float time_delta_distance_us = 11;
    total_size += 1 + 4;
  }

  if (has_distance()) {
    // required float distance = 12;
    total_size += 1 + 4;
  }

  return total_size;
}
size_t opticalFlow::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:opticalFlow_msgs.msgs.opticalFlow)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (((_has_bits_[0] & 0x00000fff) ^ 0x00000fff) == 0) {  // All required fields are present.
    // required float time_usec = 1;
    total_size += 1 + 4;

    // required float sensor_id = 2;
    total_size += 1 + 4;

    // required float integration_time_us = 3;
    total_size += 1 + 4;

    // required float integrated_x = 4;
    total_size += 1 + 4;

    // required float integrated_y = 5;
    total_size += 1 + 4;

    // required float integrated_xgyro = 6;
    total_size += 1 + 4;

    // required float integrated_ygyro = 7;
    total_size += 1 + 4;

    // required float integrated_zgyro = 8;
    total_size += 1 + 4;

    // required float temperature = 9;
    total_size += 1 + 4;

    // required float quality = 10;
    total_size += 1 + 4;

    // required float time_delta_distance_us = 11;
    total_size += 1 + 4;

    // required float distance = 12;
    total_size += 1 + 4;

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void opticalFlow::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:opticalFlow_msgs.msgs.opticalFlow)
  GOOGLE_DCHECK_NE(&from, this);
  const opticalFlow* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const opticalFlow>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:opticalFlow_msgs.msgs.opticalFlow)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:opticalFlow_msgs.msgs.opticalFlow)
    MergeFrom(*source);
  }
}

void opticalFlow::MergeFrom(const opticalFlow& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:opticalFlow_msgs.msgs.opticalFlow)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 255u) {
    if (cached_has_bits & 0x00000001u) {
      time_usec_ = from.time_usec_;
    }
    if (cached_has_bits & 0x00000002u) {
      sensor_id_ = from.sensor_id_;
    }
    if (cached_has_bits & 0x00000004u) {
      integration_time_us_ = from.integration_time_us_;
    }
    if (cached_has_bits & 0x00000008u) {
      integrated_x_ = from.integrated_x_;
    }
    if (cached_has_bits & 0x00000010u) {
      integrated_y_ = from.integrated_y_;
    }
    if (cached_has_bits & 0x00000020u) {
      integrated_xgyro_ = from.integrated_xgyro_;
    }
    if (cached_has_bits & 0x00000040u) {
      integrated_ygyro_ = from.integrated_ygyro_;
    }
    if (cached_has_bits & 0x00000080u) {
      integrated_zgyro_ = from.integrated_zgyro_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  if (cached_has_bits & 3840u) {
    if (cached_has_bits & 0x00000100u) {
      temperature_ = from.temperature_;
    }
    if (cached_has_bits & 0x00000200u) {
      quality_ = from.quality_;
    }
    if (cached_has_bits & 0x00000400u) {
      time_delta_distance_us_ = from.time_delta_distance_us_;
    }
    if (cached_has_bits & 0x00000800u) {
      distance_ = from.distance_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void opticalFlow::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:opticalFlow_msgs.msgs.opticalFlow)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void opticalFlow::CopyFrom(const opticalFlow& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:opticalFlow_msgs.msgs.opticalFlow)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool opticalFlow::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000fff) != 0x00000fff) return false;
  return true;
}

void opticalFlow::Swap(opticalFlow* other) {
  if (other == this) return;
  InternalSwap(other);
}
void opticalFlow::InternalSwap(opticalFlow* other) {
  using std::swap;
  swap(time_usec_, other->time_usec_);
  swap(sensor_id_, other->sensor_id_);
  swap(integration_time_us_, other->integration_time_us_);
  swap(integrated_x_, other->integrated_x_);
  swap(integrated_y_, other->integrated_y_);
  swap(integrated_xgyro_, other->integrated_xgyro_);
  swap(integrated_ygyro_, other->integrated_ygyro_);
  swap(integrated_zgyro_, other->integrated_zgyro_);
  swap(temperature_, other->temperature_);
  swap(quality_, other->quality_);
  swap(time_delta_distance_us_, other->time_delta_distance_us_);
  swap(distance_, other->distance_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata opticalFlow::GetMetadata() const {
  protobuf_OpticalFlow_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_OpticalFlow_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace msgs
}  // namespace opticalFlow_msgs
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::opticalFlow_msgs::msgs::opticalFlow* Arena::CreateMaybeMessage< ::opticalFlow_msgs::msgs::opticalFlow >(Arena* arena) {
  return Arena::CreateInternal< ::opticalFlow_msgs::msgs::opticalFlow >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
