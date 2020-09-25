#ifndef __UPDATE_MAGNET_PAIR__
#define __UPDATE_MAGNET_PAIR__

#include <pcs_ros/UpdateMagnetPair.h>
#include <pcs_ros/UpdateMagnetPairRequest.h>
#include <pcs_ros/UpdateMagnetPairResponse.h>

namespace SvcName {
inline std::string UpdateMagnetPair(const unsigned int& _idPoco){ return "/update_"+ std::to_string(_idPoco) +"_pair_srv"; }
}

namespace ReqType {
enum UpdateType {Close, Distant, Deleted};
}

#endif
