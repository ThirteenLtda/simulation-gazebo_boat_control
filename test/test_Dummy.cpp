#include <boost/test/unit_test.hpp>
#include <gazebo_boat_controller/Dummy.hpp>

using namespace gazebo_boat_controller;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    gazebo_boat_controller::DummyClass dummy;
    dummy.welcome();
}
