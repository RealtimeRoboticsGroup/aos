#include "frc/vision/media_device.h"

#include <fcntl.h>
#include <linux/media.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"

#include "aos/scoped/scoped_fd.h"
#include "aos/util/file.h"

namespace frc::vision {

void Entity::Log() const {
  ABSL_LOG(INFO) << "  { \"id\": " << id() << ",";
  ABSL_LOG(INFO) << "    \"name\": \"" << name() << "\",";
  ABSL_LOG(INFO) << "    \"function\": " << function() << ",";
  ABSL_LOG(INFO) << "    \"interface_type\": " << interface_type() << ",";
  ABSL_LOG(INFO) << "    \"major\": " << major() << ",";
  ABSL_LOG(INFO) << "    \"minor\": " << minor() << ",";
  if (has_interface_) {
    ABSL_LOG(INFO) << "    \"device\": \"" << device() << "\",";
  }
  ABSL_LOG(INFO) << "    \"pads\": [";
  for (const Pad *pad : pads_) {
    pad->Log();
  }
  ABSL_LOG(INFO) << "    ]";
  ABSL_LOG(INFO) << "   }";
}

void Entity::UpdateDevice() {
  // There's a symlink in /sys/dev/char which gets us to the uevent file
  // which has the DEVNAME variable set with the device name.  This
  // reliably gets us the name of the device.
  const ::std::string contents = aos::util::ReadFileToStringOrDie(
      absl::StrCat("/sys/dev/char/", major(), ":", minor(), "/uevent"));

  // Strip it out and return it.
  for (std::string_view line : absl::StrSplit(contents, "\n")) {
    ABSL_VLOG(1) << line;
    if (line.size() > 8 && line.substr(0, 8) == "DEVNAME=") {
      device_ = absl::StrCat("/dev/", line.substr(8, -1));
      return;
    }
  }

  ABSL_LOG(FATAL) << "Failed to find DEVNAME in uevent file.";
}

std::optional<MediaDevice> MediaDevice::Initialize(int index) {
  int fd = open(absl::StrCat("/dev/media", index).c_str(), O_RDWR);
  std::optional<MediaDevice> result = std::nullopt;
  if (fd >= 0) {
    result.emplace(MediaDevice(fd));
  }
  return result;
}

void MediaDevice::Update() {
  ABSL_PCHECK(ioctl(fd_.get(), MEDIA_IOC_DEVICE_INFO, &device_info_) == 0);

  struct media_v2_topology topology;
  std::memset(&topology, 0, sizeof(topology));
  ABSL_PCHECK(ioctl(fd_.get(), MEDIA_IOC_G_TOPOLOGY, &topology) == 0);
  ABSL_VLOG(1) << "Got " << topology.num_entities << " entries";
  ABSL_VLOG(1) << "Got " << topology.num_interfaces << " interfaces";
  ABSL_VLOG(1) << "Got " << topology.num_pads << " pads";
  ABSL_VLOG(1) << "Got " << topology.num_links << " links";

  std::vector<struct media_v2_entity> entities;
  entities.resize(topology.num_entities);
  topology.ptr_entities = reinterpret_cast<uint64_t>(entities.data());
  std::vector<struct media_v2_interface> interfaces;
  interfaces.resize(topology.num_interfaces);
  topology.ptr_interfaces = reinterpret_cast<uint64_t>(interfaces.data());

  std::vector<struct media_v2_pad> pads;
  pads.resize(topology.num_pads);
  topology.ptr_pads = reinterpret_cast<uint64_t>(pads.data());
  std::vector<struct media_v2_link> links;
  links.resize(topology.num_links);
  topology.ptr_links = reinterpret_cast<uint64_t>(links.data());
  ABSL_PCHECK(ioctl(fd_.get(), MEDIA_IOC_G_TOPOLOGY, &topology) == 0);

  entities_.reserve(entities.size());
  for (const struct media_v2_entity &entity : entities) {
    entities_.emplace_back();
    entities_.back().entity_ = entity;
  }

  pads_.reserve(pads.size());
  for (const struct media_v2_pad &pad : pads) {
    Entity *found_entity = nullptr;
    for (Entity &entity : entities_) {
      if (entity.id() == pad.entity_id) {
        found_entity = &entity;
        break;
      }
    }
    ABSL_CHECK(found_entity != nullptr);
    pads_.emplace_back();
    pads_.back().id_ = pad.id;
    pads_.back().flags_ = pad.flags;
    pads_.back().entity_ = found_entity;

    found_entity->pads_.emplace_back(&pads_.back());
    pads_.back().index_ = found_entity->pads_.size() - 1u;
  }

  links_.reserve(links.size());

  for (const struct media_v2_link &link : links) {
    ABSL_VLOG(1) << "Link " << link.id << " from " << link.source_id << " to "
                 << link.sink_id;
    if ((link.flags & MEDIA_LNK_FL_LINK_TYPE) == MEDIA_LNK_FL_INTERFACE_LINK) {
      const struct media_v2_interface *found_interface = nullptr;
      for (const struct media_v2_interface &interface : interfaces) {
        if (interface.id == link.source_id) {
          found_interface = &interface;
          break;
        }
      }
      ABSL_CHECK(found_interface != nullptr) << ": Failed to find interface";
      bool found = false;
      for (Entity &entity : entities_) {
        if (entity.id() == link.sink_id) {
          found = true;
          ABSL_VLOG(1) << "Added interface to " << entity.name();
          entity.has_interface_ = true;
          entity.interface_ = *found_interface;
          entity.UpdateDevice();
          break;
        }
      }
      ABSL_CHECK(found);

    } else if ((link.flags & MEDIA_LNK_FL_LINK_TYPE) ==
               MEDIA_LNK_FL_DATA_LINK) {
      links_.emplace_back();
      links_.back().flags_ = link.flags;
      links_.back().id_ = link.id;

      Pad *found_source_pad = nullptr;
      Pad *found_sink_pad = nullptr;
      for (Pad &pad : pads_) {
        if (pad.id() == link.source_id) {
          found_source_pad = &pad;
        } else if (pad.id() == link.sink_id) {
          found_sink_pad = &pad;
        }
      }
      ABSL_CHECK(found_source_pad != nullptr);
      ABSL_CHECK(found_sink_pad != nullptr);

      links_.back().source_ = found_source_pad;
      links_.back().sink_ = found_sink_pad;
      links_.back().id_ = link.id;
      found_source_pad->links_.push_back(&links_.back());
      found_sink_pad->links_.push_back(&links_.back());
    } else {
      ABSL_LOG(FATAL) << "Unknown link type " << link.flags;
    }
  }
}

void MediaDevice::Log() const {
  ABSL_LOG(INFO) << "{\"driver\": \"" << driver() << "\",";
  ABSL_LOG(INFO) << " \"model\": \"" << model() << "\",";
  ABSL_LOG(INFO) << " \"serial\": \"" << serial() << "\",";
  ABSL_LOG(INFO) << " \"bus_info\": \"" << bus_info() << "\",";
  ABSL_LOG(INFO) << " \"entities\": [";
  for (const Entity &entity : entities_) {
    entity.Log();
  }
  ABSL_LOG(INFO) << "] }";
}

void Pad::Log() const {
  ABSL_LOG(INFO) << "     {\"id\": " << id() << ",";
  ABSL_LOG(INFO) << "      \"index\": " << index() << ",";
  ABSL_LOG(INFO) << "      \"type\": \"" << (source() ? "source" : "sink")
                 << "\"";
  ABSL_LOG(INFO) << "      \"links\": [";
  for (size_t i = 0; i < links_size(); ++i) {
    ABSL_LOG(INFO) << "       {";
    if (source()) {
      ABSL_LOG(INFO) << "        \"sink\": \""
                     << links(i)->sink()->entity()->name() << "\",";
      ABSL_LOG(INFO) << "        \"sink_index\": \""
                     << links(i)->sink()->index() << "\",";
    } else {
      ABSL_LOG(INFO) << "        \"source\": \""
                     << links(i)->source()->entity()->name() << "\",";
      ABSL_LOG(INFO) << "        \"source_index\": \""
                     << links(i)->source()->index() << "\",";
    }
    ABSL_LOG(INFO) << "        \"enabled\": " << links(i)->enabled() << ",";
    ABSL_LOG(INFO) << "        \"immutable\": " << links(i)->immutable() << ",";
    ABSL_LOG(INFO) << "       }";
  }
  ABSL_LOG(INFO) << "      ],";
  ABSL_LOG(INFO) << "     }";
}

void Pad::SetSubdevCrop(uint32_t width, uint32_t height) {
  int fd = open(entity()->device().c_str(), O_RDWR);
  ABSL_PCHECK(fd >= 0);

  struct v4l2_subdev_selection selection;
  std::memset(&selection, 0, sizeof(selection));
  selection.which = V4L2_SUBDEV_FORMAT_ACTIVE;
  selection.pad = index();

  ABSL_PCHECK(ioctl(fd, VIDIOC_SUBDEV_G_SELECTION, &selection) == 0)
      << ": Failed to set " << entity()->device();

  selection.target = V4L2_SEL_TGT_CROP;
  selection.r.left = 0;
  selection.r.top = 0;
  selection.r.width = width;
  selection.r.height = height;

  ABSL_PCHECK(ioctl(fd, VIDIOC_SUBDEV_S_SELECTION, &selection) == 0);
  ABSL_LOG(INFO) << "Setting " << entity()->name() << " pad " << index()
                 << " crop to (0, 0) " << width << "x" << height;
}
void Pad::SetSubdevFormat(uint32_t width, uint32_t height, uint32_t code) {
  ABSL_VLOG(1) << "Opening " << entity()->device();
  int fd = open(entity()->device().c_str(), O_RDWR);
  ABSL_PCHECK(fd >= 0);

  struct v4l2_subdev_format format;
  std::memset(&format, 0, sizeof(format));
  format.pad = index();
  format.which = V4L2_SUBDEV_FORMAT_ACTIVE;

  ABSL_PCHECK(ioctl(fd, VIDIOC_SUBDEV_G_FMT, &format) == 0);

  ABSL_VLOG(1) << format.format.width << ", " << format.format.height << ", "
               << format.format.code << " field " << format.format.field
               << " colorspace " << format.format.colorspace << " ycbcr_enc "
               << format.format.ycbcr_enc << " quantization "
               << format.format.quantization << " xfer_func "
               << format.format.xfer_func;

  format.format.width = width;
  format.format.height = height;
  format.format.code = code;
  format.format.field = V4L2_FIELD_NONE;
  format.format.colorspace = V4L2_COLORSPACE_SRGB;
  format.format.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
  format.format.quantization = V4L2_QUANTIZATION_DEFAULT;
  format.format.xfer_func = V4L2_XFER_FUNC_DEFAULT;

  ABSL_LOG(INFO) << "Setting " << entity()->name() << " pad " << index()
                 << " format to " << width << "x" << height << " code 0x"
                 << std::hex << code;

  ABSL_PCHECK(ioctl(fd, VIDIOC_SUBDEV_S_FMT, &format) == 0);

  ABSL_PCHECK(close(fd) == 0);
}

void Entity::SetFormat(uint32_t width, uint32_t height, uint32_t code) {
  ABSL_VLOG(1) << "Opening " << device();
  int fd = open(device().c_str(), O_RDWR);
  ABSL_PCHECK(fd >= 0);

  struct v4l2_format format;
  std::memset(&format, 0, sizeof(format));
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

  ABSL_PCHECK(ioctl(fd, VIDIOC_G_FMT, &format) == 0);

  ABSL_VLOG(1) << "width " << format.fmt.pix_mp.width;
  ABSL_VLOG(1) << "height " << format.fmt.pix_mp.height;
  ABSL_VLOG(1) << "pixelformat " << format.fmt.pix_mp.pixelformat;
  ABSL_VLOG(1) << "field " << format.fmt.pix_mp.field;
  ABSL_VLOG(1) << "colorspace " << format.fmt.pix_mp.colorspace;
  ABSL_VLOG(1) << "  sizeimage " << format.fmt.pix_mp.plane_fmt[0].sizeimage;
  ABSL_VLOG(1) << "  bytesperline "
               << format.fmt.pix_mp.plane_fmt[0].bytesperline;
  ABSL_VLOG(1) << "num_planes "
               << static_cast<uint64_t>(format.fmt.pix_mp.num_planes);
  ABSL_VLOG(1) << "flags " << static_cast<uint64_t>(format.fmt.pix_mp.flags);
  ABSL_VLOG(1) << "ycbcr_enc "
               << static_cast<uint64_t>(format.fmt.pix_mp.ycbcr_enc);
  ABSL_VLOG(1) << "quantization "
               << static_cast<uint64_t>(format.fmt.pix_mp.quantization);
  ABSL_VLOG(1) << "xfer_func "
               << static_cast<uint64_t>(format.fmt.pix_mp.xfer_func);

  format.fmt.pix_mp.width = width;
  format.fmt.pix_mp.height = height;
  format.fmt.pix_mp.pixelformat = code;
  format.fmt.pix_mp.field = V4L2_FIELD_NONE;
  format.fmt.pix_mp.colorspace = V4L2_COLORSPACE_DEFAULT;
  format.fmt.pix_mp.num_planes = 1;

  // TODO(austin): This is probably V4L2_PIX_FMT_YUV422P specific...  We really
  // want to extract bytes/pixel.
  ABSL_CHECK((code == V4L2_PIX_FMT_YUV422P) || (code == V4L2_PIX_FMT_YUYV));
  format.fmt.pix_mp.plane_fmt[0].sizeimage = width * height * 2;
  format.fmt.pix_mp.plane_fmt[0].bytesperline = width;

  format.fmt.pix_mp.flags = 0;
  format.fmt.pix_mp.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
  format.fmt.pix_mp.quantization = V4L2_QUANTIZATION_DEFAULT;
  format.fmt.pix_mp.xfer_func = V4L2_XFER_FUNC_DEFAULT;

  ABSL_LOG(INFO) << "Setting " << name() << " to " << width << "x" << height
                 << " code 0x" << std::hex << code;
  ABSL_PCHECK(ioctl(fd, VIDIOC_S_FMT, &format) == 0);

  ABSL_PCHECK(close(fd) == 0);
}

void MediaDevice::Reset(Link *link) {
  ABSL_LOG(INFO) << "Disabling link " << link->source()->entity()->name()
                 << " -> " << link->sink()->entity()->name();
  struct media_link_desc link_desc;
  link_desc.source.entity = link->source()->entity()->id();
  link_desc.source.index = link->source()->index();
  link_desc.source.flags = 0;
  link_desc.sink.entity = link->sink()->entity()->id();
  link_desc.sink.index = link->sink()->index();
  link_desc.sink.flags = 0;
  link_desc.flags = link->flags() & (~MEDIA_LNK_FL_ENABLED);
  ABSL_PCHECK(ioctl(fd_.get(), MEDIA_IOC_SETUP_LINK, &link_desc) == 0);

  link->flags_ = link_desc.flags;
}

void MediaDevice::Enable(Link *link) {
  ABSL_LOG(INFO) << "Enabling link " << link->source()->entity()->name()
                 << " -> " << link->sink()->entity()->name();
  struct media_link_desc link_desc;
  link_desc.source.entity = link->source()->entity()->id();
  link_desc.source.index = link->source()->index();
  link_desc.source.flags = 0;
  link_desc.sink.entity = link->sink()->entity()->id();
  link_desc.sink.index = link->sink()->index();
  link_desc.sink.flags = 0;
  link_desc.flags = link->flags() | MEDIA_LNK_FL_ENABLED;
  ABSL_PCHECK(ioctl(fd_.get(), MEDIA_IOC_SETUP_LINK, &link_desc) == 0);

  link->flags_ = link_desc.flags;
}

Entity *MediaDevice::FindEntity(std::string_view entity_name) {
  for (Entity &entity : entities_) {
    if (entity.name() == entity_name) {
      return &entity;
    }
  }
  return nullptr;
}

Link *MediaDevice::FindLink(std::string_view source, int source_pad_index,
                            std::string_view sink, int sink_pad_index) {
  Entity *source_entity = FindEntity(source);
  Entity *sink_entity = FindEntity(sink);

  ABSL_CHECK(source_entity != nullptr) << ": Failed to find source " << source;
  ABSL_CHECK(sink_entity != nullptr) << ": Failed to find sink " << sink;

  Pad *source_pad = source_entity->pads()[source_pad_index];
  Pad *sink_pad = sink_entity->pads()[sink_pad_index];
  for (size_t i = 0; i < source_pad->links_size(); ++i) {
    if (source_pad->links(i)->sink() == sink_pad) {
      return source_pad->links(i);
    }
  }
  ABSL_LOG(FATAL) << "Failed to find link between " << source << " pad "
                  << source_pad_index << " and " << sink << " pad "
                  << sink_pad_index;
}

void MediaDevice::Reset() {
  ABSL_LOG(INFO) << "Resetting " << bus_info();

  for (Link &link : *links()) {
    if (!link.immutable()) {
      Reset(&link);
    }
  }
}

std::optional<MediaDevice> FindMediaDevice(std::string_view device) {
  for (int media_device_index = 0;; ++media_device_index) {
    std::optional<MediaDevice> media_device =
        MediaDevice::Initialize(media_device_index);
    if (!media_device) {
      return std::nullopt;
    }
    if (media_device->bus_info() == device) {
      return media_device;
    }
  }
}

}  // namespace frc::vision
