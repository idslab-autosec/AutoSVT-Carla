// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/Debug.h"
#include "carla/Memory.h"
#include "carla/sensor/RawData.h"
#include "carla/sensor/data/LidarWithFogData.h"

namespace carla {
namespace sensor {

  class SensorData;

namespace s11n {

  // ===========================================================================
  // -- LidarWithFogHeaderView --------------------------------------------------------
  // ===========================================================================

  /// A view over the header of a Lidar measurement.
  class LidarWithFogHeaderView {
    using Index = data::LidarWithFogData::Index;

  public:
    float GetHorizontalAngle() const {
      return reinterpret_cast<const float &>(_begin[Index::HorizontalAngle]);
    }

    uint32_t GetChannelCount() const {
      return _begin[Index::ChannelCount];
    }

    uint32_t GetPointCount(size_t channel) const {
      DEBUG_ASSERT(channel < GetChannelCount());
      return _begin[Index::SIZE + channel];
    }

  private:
    friend class LidarWithFogSerializer;

    explicit LidarWithFogHeaderView(const uint32_t *begin) : _begin(begin) {
      DEBUG_ASSERT(_begin != nullptr);
    }

    const uint32_t *_begin;
  };

  // ===========================================================================
  // -- LidarWithFogSerializer --------------------------------------------------------
  // ===========================================================================

  /// Serializes the data generated by Lidar sensors.
  class LidarWithFogSerializer {
  public:

    static LidarWithFogHeaderView DeserializeHeader(const RawData &data) {
      return LidarWithFogHeaderView{reinterpret_cast<const uint32_t *>(data.begin())};
    }

    static size_t GetHeaderOffset(const RawData &data) {
      auto View = DeserializeHeader(data);
      return sizeof(uint32_t) * (View.GetChannelCount() + data::LidarWithFogData::Index::SIZE);
    }

    template <typename Sensor>
    static Buffer Serialize(
        const Sensor &sensor,
        const data::LidarWithFogData &data,
        Buffer &&output);

    static SharedPtr<SensorData> Deserialize(RawData &&data);
  };

  // ===========================================================================
  // -- LidarWithFogSerializer implementation -----------------------------------------
  // ===========================================================================

  template <typename Sensor>
  inline Buffer LidarWithFogSerializer::Serialize(
      const Sensor &,
      const data::LidarWithFogData &data,
      Buffer &&output) {
    std::array<boost::asio::const_buffer, 2u> seq = {
        boost::asio::buffer(data._header),
        boost::asio::buffer(data._points)};
    output.copy_from(seq);
    return std::move(output);
  }

} // namespace s11n
} // namespace sensor
} // namespace carla