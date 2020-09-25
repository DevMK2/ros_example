#include "pcs_ros/RemoteSignal.h"

namespace TopicName {
inline std::string RemoteSignal(const std::string& _pocoName) {
    int idxNumber = _pocoName.find_last_of("_");
    return "/remote_signal"+_pocoName.substr(idxNumber);
}
}
