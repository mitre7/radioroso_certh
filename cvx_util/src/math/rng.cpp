#include <cvx/util/math/rng.hpp>
#include <boost/random/random_device.hpp>

namespace cvx { namespace util {

RNG::RNG() {
    static boost::random::random_device rd ;
    generator_.seed(rd);
}

RNG::RNG(uint64_t seed): generator_(seed) {

}


}}
