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
        //std::cout << "1" << std::endl;
        std::vector<rix::msg::geometry::TransformStamped> still_to_go = is_above(tf, link_name, reference_link);
        //std::cout << "2" << std::endl;
        if(still_to_go.size() > 0){
            //std::cout << "3" << std::endl;
            return solve_up(still_to_go, link_name, reference_link);
        }

        
        rix::msg::geometry::TransformStamped tstamp;
        tstamp.child_frame_id = "none";
        
        for(rix::msg::geometry::TransformStamped cur_tstamp : tf.transforms){
            if(cur_tstamp.child_frame_id == link_name){
                tstamp = cur_tstamp;
            }
        }
        if(tstamp.child_frame_id == "none"){
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
        if(tstamp.header.frame_id == reference_link){
            return aff;
        }
        return solve(tf, tstamp.header.frame_id, reference_link) * aff;
    }

    return Eigen::Affine3d::Identity();
}

std::vector<rix::msg::geometry::TransformStamped> FKSolver::is_above(const rix::msg::geometry::TF &tf, const std::string &link_name, const std::string reference_link){
    std::vector<rix::msg::geometry::TransformStamped> transform_of_interest;
    for(auto transform : tf.transforms){
        //std::cout << "10" << std::endl;
        if(transform.header.frame_id == link_name){
            //std::cout << "11" << std::endl;
            transform_of_interest.push_back(transform);
            //std::cout << "12" << std::endl;
        }
    }
    //std::cout << "13" << std::endl;
    for(auto transform : transform_of_interest){
        //std::cout << "14" << std::endl;
        std::vector<rix::msg::geometry::TransformStamped> route;
        //std::cout << "15" << std::endl;
        route = is_above(tf, transform.child_frame_id, reference_link);
        //std::cout << "16" << std::endl;
        if(route.size() != 0){
            //std::cout << "17" << std::endl;
            return route;
        }
    }
    //std::cout << "18" << std::endl;
    std::vector<rix::msg::geometry::TransformStamped> empty_vec;
    return empty_vec; 
    
}

Eigen::Affine3d FKSolver::solve_up(std::vector<rix::msg::geometry::TransformStamped> &tf, const std::string &link_name,
    const std::string &reference_link) {
        //std::cout << "20" << std::endl;
    if(link_name != reference_link){
        //std::cout << "21" << std::endl;
        rix::msg::geometry::TransformStamped tstamp;
        //std::cout << "22" << std::endl;
        std::vector<rix::msg::geometry::TransformStamped> still_to_go;
        //std::cout << "23" << std::endl;
        tstamp.header.frame_id = "none";
        
        for(rix::msg::geometry::TransformStamped cur_tstamp : tf){
            //std::cout << "24" << std::endl;
            if(cur_tstamp.header.frame_id == link_name){
                //std::cout << "25" << std::endl;
                tstamp = cur_tstamp;
            }
            else{
                //std::cout << "26" << std::endl;
                still_to_go.push_back(cur_tstamp);
            }
        }
        //std::cout << "27" << std::endl;
        if(tstamp.header.frame_id == "none"){
            //std::cout << "28" << std::endl;
            return Eigen::Affine3d::Identity();
        }
        
        //std::cout << "29" << std::endl;
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
        aff = translation * rot * Eigen::Scaling(-1.0);
        if(tstamp.child_frame_id == reference_link){
            return aff;
        }
        //std::cout << "30" << std::endl;
        return aff *  solve_up(still_to_go, tstamp.child_frame_id, reference_link) ;



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