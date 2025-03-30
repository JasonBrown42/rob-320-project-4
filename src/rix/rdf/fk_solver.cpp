#include <rix/rdf/fk_solver.hpp>

namespace rix {
namespace rdf {

FKSolver::FKSolver(const Tree &tree) : tree(tree) {}

Eigen::Affine3d FKSolver::solve(const std::string &link_name) const {
    /**
     * TODO: Return the global transformation of the link.
     */

    Tree test_tree = tree;
    if(link_name != "world"){
        const rix::msg::geometry::TF tf = (const rix::msg::geometry::TF) test_tree.TF();
        return solve(tf, link_name, "world");
    }

    return Eigen::Affine3d::Identity();
}

Eigen::Affine3d FKSolver::solve(const std::string &link_name, const std::string &reference_link) const {
    /**
     * TODO: Return the transformation of the link relative to the reference link.
     */
     Tree test_tree = tree;

    if(link_name != reference_link){
        const rix::msg::geometry::TF tf = (const rix::msg::geometry::TF) test_tree.TF();
        return solve(tf, link_name, reference_link);
    }

    return Eigen::Affine3d::Identity();
}

Eigen::Affine3d FKSolver::solve(const rix::msg::geometry::TF &tf, const std::string &link_name) {
    /**
     * TODO: Given a TF message, return the global transformation of the link.
     */
    if(link_name != "world"){
        return solve(tf, link_name, "world");
    }
    return Eigen::Affine3d::Identity();
}

Eigen::Affine3d FKSolver::solve(const rix::msg::geometry::TF &tf, const std::string &link_name,
                                const std::string &reference_link) {
    /**
     * TODO: Given a TF message, return the transformation of the link relative to the reference link.
     */
    if(link_name != reference_link){
        rix::msg::geometry::TransformStamped tstamp;
        tstamp.child_frame_id = "none";
        
        for(rix::msg::geometry::TransformStamped cur_tstamp : tf.transforms){
            if(cur_tstamp.child_frame_id == link_name){
                tstamp = cur_tstamp;
            }
        }
        if(tstamp.child_frame_id == "none" || tstamp.child_frame_id == reference_link){
            return Eigen::Affine3d::Identity();
        }
        

        Eigen::Translation3d translation;
        translation.x() = tstamp.transform.translation.x;
        translation.y() = tstamp.transform.translation.y;
        translation.z() = tstamp.transform.translation.z;

        Eigen::Quaterniond quat;
        quat.w() = tstamp.transform.rotation.w;
        quat.x() = tstamp.transform.rotation.x;
        quat.y() = tstamp.transform.rotation.y;
        quat.z() = tstamp.transform.rotation.z;
        Eigen::Matrix3d rot(quat);

        Eigen::Affine3d aff;
        aff = translation * rot;

        return solve(tf, tstamp.header.frame_id, reference_link) * aff;
    }

    return Eigen::Affine3d::Identity();
}

rix::msg::geometry::TF FKSolver::global_tf() const {
    /**
     * TODO: Generate a global TF message from the tree. This message should
     * contain the global transformation of each link in the tree.
     */
    return rix::msg::geometry::TF();
}

rix::msg::geometry::TF FKSolver::global_tf(const rix::msg::geometry::TF &tf) {
    /**
     * TODO: Given a TF message, return a new TF message containing the global
     * transformation of each link in the input message.
     */
    return rix::msg::geometry::TF();
}

}  // namespace rdf
}  // namespace rix