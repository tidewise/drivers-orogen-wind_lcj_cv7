/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#include <marnav/nmea/checksum.hpp>
#include <marnav/nmea/mwv.hpp>
#include <marnav/nmea/xdr.hpp>

#include <base-logging/Logging.hpp>

using namespace marnav;
using namespace std;
using namespace wind_lcj_cv7;

Task::Task(string const& name)
    : TaskBase(name)
{
}

Task::~Task()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::processRawSentence(string const& sentence_string) {
    if (sentence_string.substr(3, 3) != "XDR") {
        return TaskBase::processRawSentence(sentence_string);
    }

    // we have to remove the last comma in the string
    size_t last_comma = sentence_string.find_last_of(",");

    string new_sentence = sentence_string.substr(0, last_comma);
    int checksum = marnav::nmea::checksum(new_sentence.begin() + 1, new_sentence.end());
    return TaskBase::processRawSentence(
        new_sentence + "*" + marnav::nmea::checksum_to_string(checksum)
    );
}
bool Task::processSentence(marnav::nmea::sentence const& sentence) {
    if (sentence.id() == nmea::sentence_id::MWV) {
        auto mwv = nmea::sentence_cast<nmea::mwv>(&sentence);
        processMWV(*mwv);
        return true;
    }
    else if (sentence.id() == nmea::sentence_id::XDR) {
        auto xdr = nmea::sentence_cast<nmea::xdr>(&sentence);
        processXDR(*xdr);
        return true;
    }
    else {
        return false;
    }
}
void Task::processMWV(marnav::nmea::mwv const& mwv) {
    auto valid = mwv.get_data_valid();
    if (!valid || *valid != nmea::status::ok) {
        return;
    }
    if (!mwv.get_angle() || !mwv.get_speed()) {
        return;
    }

    double angle = (-*mwv.get_angle() + 180) * M_PI / 180;
    double speed = *mwv.get_speed();

    auto unit = *mwv.get_speed_unit();
    double factor = 1;
    if (unit == nmea::unit::velocity::knot) {
        factor = 0.514444;
    }
    else if (unit == nmea::unit::velocity::kmh) {
        factor = 1.0 / 3.6;
    }

    base::samples::RigidBodyState air_speed;
    air_speed.velocity = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ())
                         * Eigen::Vector3d(speed * factor, 0, 0);
    air_speed.time = base::Time::now();
    _air_speed.write(air_speed);
}
void Task::processXDR(marnav::nmea::xdr const& xdr) {
    auto info = xdr.get_info(0);
    if (!info) {
        return;
    }

    auto temperature = base::samples::Temperature::fromCelsius(
        base::Time::now(), info->measurement_data
    );
    _air_temperature.write(temperature);
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
