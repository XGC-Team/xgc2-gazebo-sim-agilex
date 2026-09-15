#include "scout_gazebo/ode_contact_feedback.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace contact = scout_gazebo::ode_contact;
using Matrix = std::array<contact::Vector, 3>; // Row-major in this test only.
static unsigned checks = 0;
#define CHECK(condition) do { ++checks; if (!(condition)) \
    throw std::runtime_error(std::string(#condition) + " at line " + std::to_string(__LINE__)); } while (false)

static Matrix multiply(const Matrix& a, const Matrix& b) {
    Matrix out{};
    for (unsigned r=0;r<3;++r) for (unsigned c=0;c<3;++c)
        for (unsigned k=0;k<3;++k) out[r][c]+=a[r][k]*b[k][c];
    return out;
}
static Matrix rotation(double roll, double pitch, double yaw) {
    const double cr=std::cos(roll),sr=std::sin(roll);
    const double cp=std::cos(pitch),sp=std::sin(pitch);
    const double cy=std::cos(yaw),sy=std::sin(yaw);
    const Matrix rx{{{{1,0,0}},{{0,cr,-sr}},{{0,sr,cr}}}};
    const Matrix ry{{{{cp,0,sp}},{{0,1,0}},{{-sp,0,cp}}}};
    const Matrix rz{{{{cy,-sy,0}},{{sy,cy,0}},{{0,0,1}}}};
    return multiply(multiply(rz,ry),rx);
}
static contact::Frame frame(const Matrix& r, double time=12.0) {
    contact::Frame f;
    f.valid=true;f.time_s=time;
    for(unsigned c=0;c<3;++c)for(unsigned i=0;i<3;++i)f.columns[c][i]=r[i][c];
    return f;
}
static contact::Vector local(const Matrix& r, const contact::Vector& world) {
    contact::Vector out{};
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)out[i]+=r[j][i]*world[j];
    return out;
}
static bool close(double a,double b) { return std::abs(a-b)<1e-10; }
static void test_rotations_and_body_selection() {
    const double pi=std::acos(-1.0);
    const contact::Vector expected{{3,-4,68}},up{{0,0,1}},other{{-901,233,-17}};
    // Both installation sides, full wheel revolutions and chassis yaw. Body2
    // must use the wheel frame, not the (unrelated) other body's frame/sign.
    for(double mount : {-pi/2,pi/2}) for(double spin : {-pi,-1.2,0.0,.4,pi/2,pi})
        for(double yaw : {0.0,.7,-2.0}) for(bool first : {true,false}) {
            const auto r=rotation(mount,spin,yaw);
            const auto raw=local(r,expected);
            const auto out=contact::restore(frame(r),12,12,first,first?raw:other,first?other:raw,up);
            CHECK(out.rejection==contact::Rejection::none);
            CHECK(close(out.normal_n,68));
            for(unsigned k=0;k<3;++k) { CHECK(close(out.world[k],expected[k]));CHECK(close(out.raw[k],raw[k])); }
        }
    // The reported failure: a horizontal wheel-link Z need not contain any N.
    const auto r=rotation(pi/2,0,0);
    const auto raw=local(r,{{0,0,68}});
    CHECK(std::abs(raw[2])<1e-10);
    CHECK(close(contact::restore(frame(r),12,12,true,raw,other,up).normal_n,68));
}
static void test_pre_step_not_latest_rotation() {
    const contact::Vector world{{0,0,68}},up{{0,0,1}},unused{{0,0,0}};
    const auto before=rotation(.3,.7,.2);
    const auto after=rotation(.3,1.2,.2);
    const auto raw=local(before,world);
    const auto good=contact::restore(frame(before),12,12,true,raw,unused,up);
    const auto wrong=contact::restore(frame(after),12,12,true,raw,unused,up);
    CHECK(close(good.normal_n,68));
    CHECK(std::abs(wrong.normal_n-good.normal_n)>1.0);
}
static void test_no_synthetic_support_and_nonfinite() {
    const auto f=frame(rotation(0,0,0));
    const contact::Vector up{{0,0,1}},unused{{0,0,0}};
    for(const auto& raw : {contact::Vector{{0,0,0}},contact::Vector{{0,0,-68}},contact::Vector{{1000,500,0}}}) {
        const auto out=contact::restore(f,12,12,true,raw,unused,up);
        CHECK(out.rejection==contact::Rejection::nonpositive);CHECK(out.normal_n==0);
    }
    for(double bad : {std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()})
        for(unsigned axis=0;axis<3;++axis) {
            contact::Vector raw{{0,0,68}};raw[axis]=bad;
            const auto out=contact::restore(f,12,12,true,raw,unused,up);
            CHECK(out.rejection==contact::Rejection::nonfinite);CHECK(out.normal_n==0);
            // Do not accidentally inspect the unselected body's invalid data.
            CHECK(close(contact::restore(f,12,12,false,raw,{{0,0,68}},up).normal_n,68));
        }
    auto bad_frame=f;bad_frame.columns[0][0]=std::numeric_limits<double>::quiet_NaN();
    CHECK(contact::restore(bad_frame,12,12,true,{{0,0,68}},unused,up).rejection==contact::Rejection::nonfinite);
    const double big=std::numeric_limits<double>::max();
    CHECK(contact::restore(frame(rotation(0,0,-.785)),12,12,true,{{big,big,big}},unused,up).rejection==contact::Rejection::nonfinite);
    CHECK(close(contact::restore(f,12,12,true,{{3,4,0}},unused,{{.6,.8,0}}).normal_n,5));
}
static void test_step_stamp_and_epoch_invalidation() {
    const contact::Vector raw{{0,0,68}},unused{{0,0,0}},up{{0,0,1}};
    for(double h : {.001,.002,.004}) {
        auto f=frame(rotation(0,0,0));
        CHECK(close(contact::restore(f,12,12,true,raw,unused,up).normal_n,68));
        CHECK(contact::restore(f,12-h,12,true,raw,unused,up).rejection==contact::Rejection::wrong_time);
        CHECK(contact::restore(f,12,12+h,true,raw,unused,up).rejection==contact::Rejection::wrong_time);
        CHECK(contact::restore(f,0,0,true,raw,unused,up).rejection==contact::Rejection::wrong_time);
        f={}; // The production pause/reset/consume path invalidates this way.
        CHECK(contact::restore(f,12,12,true,raw,unused,up).rejection==contact::Rejection::missing_frame);
        f=frame(rotation(0,0,0),0);
        CHECK(close(contact::restore(f,0,0,true,raw,unused,up).normal_n,68));
    }
    const auto f=frame(rotation(0,0,0));
    const double nan=std::numeric_limits<double>::quiet_NaN();
    CHECK(contact::restore(f,nan,12,true,raw,unused,up).rejection==contact::Rejection::nonfinite);
    CHECK(contact::restore(f,12,nan,true,raw,unused,up).rejection==contact::Rejection::nonfinite);
}
int main() {
    try {
        test_rotations_and_body_selection();test_pre_step_not_latest_rotation();
        test_no_synthetic_support_and_nonfinite();test_step_stamp_and_epoch_invalidation();
        std::cout<<"ODE contact adapter: "<<checks<<" checks passed\n";
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
