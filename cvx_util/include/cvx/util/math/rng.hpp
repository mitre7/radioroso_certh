#ifndef __RNG_HPP__
#define __RNG_HPP__

#include <boost/random.hpp>
#include <boost/random/random_device.hpp>
#include <boost/range/algorithm.hpp>

namespace cvx { namespace util {

class RNG {
public:
    RNG() ;
    RNG(uint64_t seed) ;

    template <class T>
    T uniform(T min_v, T max_v) {
        assert(boost::is_integral<T>()) ;
        boost::random::uniform_int_distribution<> dis(min_v, max_v);
        return dis(generator_) ;
    }

    // uniform random number in [0, 1)

    double uniform() {
        boost::random::uniform_real_distribution<> dis(0, 1);
        return dis(generator_) ;
    }

    double gaussian(double mean, double sigma) {
        boost::random::normal_distribution<double> dis(mean, sigma);
        return dis(generator_) ;
    }

    float gaussian(float mean, float sigma) {
        boost::random::normal_distribution<float> dis(mean, sigma);
        return dis(generator_) ;
    }

    // gaussian with mean 0 and variance 1

    double gaussian() {
        boost::random::normal_distribution<double> dis(0, 1);
        return dis(generator_) ;
    }

    // generate a random sequence of N unique integers

    template <class T>
    void sequence(uint64_t N, std::vector<T> &seq) {
        assert(boost::is_integral<T>()) ;
        for( uint64_t i=0 ; i<N ; i++ ) seq.push_back(T(i)) ;
        shuffle(seq) ;
    }

    // random shuffling of a sequence
    template <class T>
    void shuffle(std::vector<T> &seq) {
        boost::uniform_int<uint64_t> uni_dist;
        boost::variate_generator<rng_t&, boost::uniform_int<uint64_t> > rnd(generator_, uni_dist);
        boost::range::random_shuffle(seq, rnd) ;
    }

    // sample without replacement n numbers from the set [0, N)
    template<class T>
    void sample(uint32_t n, uint64_t N, std::vector<T> &subset) {

        uint64_t max_idx = N-1 ;

        std::vector<T> vidx ;
        for( uint64_t i=0 ; i<N ; i++ ) vidx.push_back(i) ;

        for ( int i=0; i<n ; i++ ) {
            boost::random::uniform_int_distribution <> ud(0, max_idx) ;
            int index = ud(generator_) ;
            std::swap(vidx[index], vidx[max_idx]);
            subset.push_back(vidx[max_idx]);
            max_idx -- ;
        }
    }

//    template<>
    float uniform(float x, float y) {
         boost::random::uniform_real_distribution<> dis(x, y);
         return dis(generator_) ;
    }

//    template<>
    double uniform(double x, double y) {
        boost::random::uniform_real_distribution<> dis(x, y);
        return dis(generator_) ;
    }


private:
    typedef boost::random::mt19937_64 rng_t ;
    rng_t generator_ ;
};



}}

#endif
