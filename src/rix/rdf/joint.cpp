#include "rix/rdf/joint.hpp"

namespace rix {
namespace rdf {

Joint::Joint(const Eigen::Vector3d &axis, const Eigen::Affine3d &origin, const JointType &type,
             const std::string &parent, const std::string &child, double upper_bound, double lower_bound)
    : axis(axis),
      origin(origin),
      type(type),
      lower_bound(lower_bound),
      upper_bound(upper_bound),
      q(0.0),
      parent(parent),
      child(child) {}

Joint::Joint(const Joint &j)
    : axis(j.axis),
      origin(j.origin),
      type(j.type),
      lower_bound(j.lower_bound),
      upper_bound(j.upper_bound),
      parent(j.parent),
      child(j.child),
      q(j.q) {}

Joint &Joint::operator=(Joint j) {
    if (this == &j) {
        return *this;
    }
    axis = j.axis;
    origin = j.origin;
    type = j.type;
    lower_bound = j.lower_bound;
    upper_bound = j.upper_bound;
    parent = j.parent;
    child = j.child;
    q = j.q;
    return *this;
}

bool Joint::is_in_bounds(double q) const { return q >= lower_bound && q <= upper_bound; }

double Joint::clamp(double q) const {
    if (q < lower_bound) {
        return lower_bound;
    } else if (q > upper_bound) {
        return upper_bound;
    }
    return q;
}

std::string Joint::get_parent() const { return parent; }

std::string Joint::get_child() const { return child; }

JointType Joint::get_type() const { return type; }

double Joint::get_lower_bound() const { return lower_bound; }

double Joint::get_upper_bound() const { return upper_bound; }

const Eigen::Vector3d &Joint::get_axis() const { return axis; }

const Eigen::Affine3d &Joint::get_origin() const { return origin; }

double Joint::get_state() const { return q; }

Eigen::Affine3d Joint::get_transform() const {
    /**
     * TODO: Get the transform of the joint.
     */
    Eigen::Affine3d out;
    out = Eigen::Affine3d::Identity();

    if(type == FIXED){
        //Fixed code here
        out = out;
    }
    else if(type == REVOLUTE){
        //revolute code here
        Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();
        mat = Eigen::AngleAxisd(q, axis);        
        out = out*mat;
    }
    else if(type == PRISMATIC){    
        //prismatic code here
        Eigen::Translation3d trans = Eigen::Translation3d(axis * Eigen::Scaling(q));
        out = trans * out;
    }
    else if(type == CONTINUOUS){
        double q_temp = q;
        while(q_temp < 0 || q_temp > 2.0*M_PI){
            if(q_temp < 0){
                q_temp += 2.0*M_PI;
            }
            else{
                q_temp -= 2.0*M_PI;
            }
        }
        Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();
        mat = Eigen::AngleAxisd(q_temp, axis);        
        out = out*mat;
    }

    
    
    // Eigen::Affine3d rot_aff = mat;
    // out = origin;//* mat;//rot_aff;
    return out;
    //Eigen::Affine3d::Identity();
}

void Joint::set_state(double q) { this->q = q; }

}  // namespace rdf
}  // namespace rix