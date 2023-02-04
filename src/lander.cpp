#include <mav_planning/lander.hpp>

using namespace _Lander_;

// Constructor
Lander::Lander(ros::NodeHandle& nh, Environment* _env): environment(_env),
	imageSubscriber(nh.subscribe("/t265_1/fisheye2/image_raw", 1, &Lander::imageCallback, this)),
	infoSubscriber(nh.subscribe("/t265_1/fisheye2/camera_info", 1, &Lander::infoCallback, this)),
    imagePublisher(nh.advertise<sensor_msgs::Image>("/debug_grid", 1)){
	
	nh.param("publish_debug_lan", debug_flag, true);
	nh.param("land_velocity", land_vv, 0.1);
	nh.param("final_zz", end_zz, -3.0);

	nh.param("theta_thr", th_thr, 0.3);
	nh.param("rho_thr", rh_thr, 50.0);
	nh.param("orthogonal_thr", ortho_thr, 0.15);

	nh.param("grid_dimension", ll_mm, 1.0);
	nh.param("square_thr_perc", thr_perc, 0.15);

    spline = new BSpline<Eigen::Matrix<double, 3, 1>>(7);
    optimizer = new Optimizer(nh, environment);
}

// Destructor
Lander::~Lander(){
	delete spline;
    delete optimizer;
}

void Lander::reset(){
    spline->clearAll();
}

bool Lander::land(std::vector<Eigen::Matrix<double, 3, 1>>& cp, std::vector<double>& cpy, double& tt, double dt){
	if(!environment->getLastOdomTransform(bodyToOdom_transformation)){
        ROS_ERROR("[LANDER]: No Odometry Received Yet");
		return false;
	}

    if(!environment->getLastTransform(odomToMap_transformation)){
        ROS_ERROR("[LANDER]: No TF Received Yet");
        return false;
    }

    bool returnValue = false;

    tf2::Transform bodyToMap_transformation = odomToMap_transformation*bodyToOdom_transformation;
    tf2::Vector3 bodyToMap_translation = bodyToMap_transformation.getOrigin();
    tf2::Quaternion bodyToMap_rotation = bodyToMap_transformation.getRotation();

    double roll, pitch;
    tf2::Matrix3x3 m(bodyToMap_rotation);
    m.getRPY(roll, pitch, actualYaw);

    actualPosition(0) = bodyToMap_translation.x();
    actualPosition(1) = bodyToMap_translation.y();
    actualPosition(2) = bodyToMap_translation.z();

    if(!image_valid && spline->getControl().size() == 0){
        ROS_WARN_COND(debug_flag, "[LANDER]: Performing Blind Land");

        // Return a Simple Landing Trajectory
        simpleLanding();

        cp = spline->getControl();
        tt = spline->getTime();

        // Yaw Trajectory Executes in Half Time!
        cpy.insert(cpy.end(), 9, actualYaw);

        returnValue = true;
        returnValue = true;
    }

    std::vector<cv::Point> pps;
    if(processImage(pps)){
        computeErrorSquare(pps);

        if(checkSquare()){
            std::vector<std::vector<Eigen::Matrix<double, 3, 1>>> bb;
            std::vector<Eigen::Matrix<double, 3, 1>> vv;
            std::vector<Eigen::Matrix<double, 3, 1>> aa;
            std::vector<Eigen::Matrix<double, 3, 1>> jj;

            std::vector<Eigen::Matrix<double, 3, 1>> wp;
            std::vector<double> wp_tt;

            Eigen::Matrix<double, 3, 1> zero = Eigen::MatrixXd::Zero(3, 1);
            Eigen::Matrix<double, 3, 1> pi;

            if(spline->getControl().size() != 0){
                pi = spline->evaluateCurve(dt, 0);

                wp.push_back(pi);
                wp_tt.push_back(0.0);

                vv.push_back(spline->evaluateCurve(dt, 1));
                vv.push_back(zero);
                aa.push_back(spline->evaluateCurve(dt, 2));
                aa.push_back(zero);
                jj.push_back(spline->evaluateCurve(dt, 3));
                jj.push_back(zero);

            } else{
                wp.push_back(actualPosition);
                wp_tt.push_back(0.0);

                vv.push_back(zero);
                vv.push_back(zero);
                aa.push_back(zero);
                aa.push_back(zero); 
                jj.push_back(zero);
                jj.push_back(zero);
	        }

            bb.push_back(vv);
            bb.push_back(aa);
            bb.push_back(jj);

            // Add Second and Last WP
            Eigen::Matrix<double, 3, 1> wpi_2;
            Eigen::Matrix<double, 3, 1> wpi_3;

            double dt_ee = sqrt(pow(ee_xx_mm, 2) + pow(ee_yy_mm, 2))/(land_vv*2);

            wpi_2 << new_xx_position, new_yy_position, actualPosition(2) - vv[0].norm()*dt_ee;
            wpi_3 << new_xx_position, new_yy_position, end_zz;

            wp.push_back(wpi_2);
            wp.push_back(wpi_3);

            tt = dt_ee + (wpi_2 - wpi_3).norm()/land_vv;
            
            wp_tt.push_back(dt_ee);
            wp_tt.push_back(tt);
            
	        spline->setWaypoints(wp, bb, tt);
            cp = spline->getControl();

            if(cp.size() != 0){
                optimizer->setWayPoints(wp, wp_tt);
                optimizer->optimizeBSpline(cp, tt, Optimizer::NORMAL_PHASE | Optimizer::WAYPOINTS);
                spline->setControl(cp, tt);
            } else{
                spline->clearAll();
            }

            // Handle Yaw Trajectory
            // Yaw Trajectory Executes in Half Time!!
            cpy.insert(cpy.end(), 3, actualYaw);
            cpy.insert(cpy.end(), 3, actualYaw - mm_px);
            cpy.insert(cpy.end(), 3, actualYaw - mm_px);

            returnValue = true;
        }

    } 
    
    if(returnValue == false && spline->getControl().size() == 0){
        ROS_WARN_COND(debug_flag, "[LANDER]: Performing Blind Land");

        // Return a Simple Landing Trajectory
        simpleLanding();

        cp = spline->getControl();
        tt = spline->getTime();

        // Yaw Trajectory Executes in Half Time!!
        cpy.insert(cpy.end(), 9, actualYaw);

        returnValue = true;
    }

    return returnValue;
}

bool Lander::processImage(std::vector<cv::Point>& pps){
    processing_image = true;
    cv::Mat img_ = img;
    processing_image = false;

    cx = (int)(img.cols/2);
    cy = (int)(img.rows/2);

    // Equalization & Thresholding
    cv::Mat img_tmp = img_.clone();
    cv::equalizeHist(img_tmp, img_tmp);
    cv::threshold(img_tmp, img_tmp, 240, 255, cv::THRESH_BINARY);

    // De-noising
    cv::GaussianBlur(img_tmp, img_tmp, cv::Size(5, 5), 0);

    // Edge Extraction
    cv::Canny(img_tmp, img_tmp, 100, 260);

    // Lines Extraction
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(img_tmp, lines, 1, CV_PI/180, 160, 0, 0);

    std::vector<cv::Vec2f> lines_;
    for(uint ii = 0; ii < lines.size(); ii++){
        double th1 = lines[ii][1];
        double rho1 = lines[ii][0];

        if(lines_.size() != 0){
            bool found_sim = false;
            for(uint jj = 0; jj < lines_.size(); jj++){
                double th2 = lines_[jj][1];
                double rho2 = lines_[jj][0];

                if(abs(th1-th2) < th_thr && abs(rho1-rho2) < rh_thr){
                    found_sim = true;
                    break;
                }
            }

            if(found_sim){
                continue;
            }
        }

        lines_.push_back(lines[ii]);
    }

    // Find Out All Intersections
    for(uint ii = 0; ii < lines_.size(); ii++){
        for(uint jj = ii; jj < lines_.size(); jj++){
            if(ii == jj){
                continue;
            }

            // Check Orthogonality
            double th1 = lines_[ii][1];
            double th2 = lines_[jj][1];

            th1 = th1 > M_PI ? th1-M_PI:th1;
            th2 = th2 > M_PI ? th2-M_PI:th2;

            th1 = th1 < 0.0 ? th1+M_PI:th1;
            th2 = th2 < 0.0 ? th2+M_PI:th2;

            if(abs(abs(th1-th2) - M_PI/2) > ortho_thr){
                continue;
            }

            // Extract Intersection
            cv::Point pp = findIntersection(lines_[ii], lines_[jj]);
            if(pp.x < 0 || pp.x > img_tmp.cols || pp.y < 0 || pp.y > img_tmp.rows){
                // Intersection Out of Image Boundaries
                continue;
            }

            pps.push_back(pp);
        }
    }

    // Find Minimum Square !!
    bool squareFound = findSquare(pps, idx1, idx2, idx3, idx4);

    if(debug_flag){
        for(uint ii = 0; ii < lines.size(); ii++){
            double rho = lines[ii][0];
            double theta = lines[ii][1];

            cv::Point pt1, pt2;
            double a = cos(theta);
            double b = sin(theta);

            double x0 = a*rho;
            double y0 = b*rho;

            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));

            //cv::line(img_, pt1, pt2, cv::Scalar(0,0,0), 3, cv::LINE_AA);
        }

        for(uint ii = 0; ii < pps.size(); ii++){
            cv::circle(img_, pps[ii], 5, cv::Scalar(0,0,0), 5);
        }

        if(squareFound){
            cv::circle(img_, pps[idx1], 10, cv::Scalar(0,0,0), 10);
            cv::circle(img_, pps[idx2], 10, cv::Scalar(0,0,0), 10);
            cv::circle(img_, pps[idx3], 10, cv::Scalar(0,0,0), 10);
            cv::circle(img_, pps[idx4], 10, cv::Scalar(0,0,0), 10);

            cv::circle(img_, cv::Point(scx,scy), 6, cv::Scalar(255,255,255), 6);
        } else{
            ROS_INFO("[LANDER]: Square Not Found!!");
        }

        // Frame Centre
        cv::line(img_, cv::Point(cx-10,cy),cv::Point(cx+10,cy), cv::Scalar(0,0,0), 3, cv::LINE_AA);
        cv::line(img_, cv::Point(cx,cy-10),cv::Point(cx,cy+10), cv::Scalar(0,0,0), 3, cv::LINE_AA);

        std_msgs::Header header;
        header.stamp = ros::Time::now();

        cv_bridge::CvImage bridge(header, sensor_msgs::image_encodings::MONO8, img_);
        imagePublisher.publish(bridge.toImageMsg());
    }

    image_valid = false;
    return squareFound;
}

bool Lander::findSquare(std::vector<cv::Point> pps, uint& idx1, uint& idx2, uint& idx3, uint& idx4){
    if(pps.size() < 4){
        return false;
    }

    std::vector<DistanceIdx> dist_with_idx;
    std::vector<std::vector<double>> dists_from_points;

    dist_with_idx.resize(pps.size());
    dists_from_points.resize(pps.size());

    for(uint ii = 0; ii < dists_from_points.size(); ii++){
        dists_from_points[ii].resize(pps.size());
    }

    for(uint ii = 0; ii < pps.size(); ii++){
        dist_with_idx[ii].dd = sqrt(pow(cx - pps[ii].x, 2) + pow(cy - pps[ii].y, 2));
        dist_with_idx[ii].idx = ii;
        
        for(uint jj = ii; jj < pps.size(); jj++){
            if(ii == jj){
                dists_from_points[ii][jj] = 0.0;
                continue;    
            }

            dists_from_points[ii][jj] = sqrt(pow(pps[jj].x - pps[ii].x, 2) + pow(pps[jj].y - pps[ii].y, 2));
            dists_from_points[jj][ii] = dists_from_points[ii][jj];
        }
    }

    // Sort Distances
    std::sort(dist_with_idx.begin(), dist_with_idx.end(), Lander::sortDistances);

    for(uint imx = 3; imx < dist_with_idx.size(); imx++){
        std::vector<DistanceIdx> pps_dd;
        pps_dd.insert(pps_dd.begin(), dist_with_idx.begin(), dist_with_idx.begin()+imx);

        std::vector<std::vector<DistanceIdx>> combined;
        combineElements(pps_dd, combined);

        for(uint ii = 0; ii < combined.size(); ii++){
            uint criterion_count = 0;
            for(uint jj = 0; jj < combined[ii].size(); jj++){
                std::vector<double> dds;
                for(uint kk = 0; kk < combined[ii].size(); kk++){
                    if(jj != kk){
                        dds.push_back(dists_from_points[combined[ii][jj].idx][combined[ii][kk].idx]);
                    }
                }

                // Check Similarity Btw Distances
                bool check_ok = false;
                for(uint dd_idx1 = 0; dd_idx1 < dds.size(); dd_idx1++){
                    for(uint dd_idx2 = 0; dd_idx2 < dds.size(); dd_idx2++){
                        if(dd_idx1 != dd_idx2){
                            if(abs(dds[dd_idx1] - dds[dd_idx2]) < dds[dd_idx1]*thr_perc){
                                // Check Diagonal Distance
                                for(uint dd_idx3 = 0; dd_idx3 < dds.size(); dd_idx3++){
                                    if(dd_idx3 != dd_idx1 && dd_idx3 != dd_idx2){
                                        if(abs(((dds[dd_idx1] + dds[dd_idx2])/2.0)*1.4142 - dds[dd_idx3]) < dds[dd_idx3]*thr_perc){
                                            // Criterion Satisfied !!
                                            check_ok = true;
                                        }
                                    }
                                }
                            }
                        }
                        
                        if(check_ok){
                            break;
                        }
                    }

                    if(check_ok){
                        break;
                    }
                }

                if(check_ok){
                    criterion_count++;
                }
            }

            if(criterion_count == 4){
                // Square Found!!
                idx1 = combined[ii][0].idx;
                idx2 = combined[ii][1].idx;
                idx3 = combined[ii][2].idx;
                idx4 = combined[ii][3].idx;

                // Compute ll & mm
                double dd12 = sqrt(pow(pps[idx1].x - pps[idx2].x, 2) + pow(pps[idx1].y - pps[idx2].y, 2));
                double dd13 = sqrt(pow(pps[idx1].x - pps[idx3].x, 2) + pow(pps[idx1].y - pps[idx3].y, 2));
                double dd14 = sqrt(pow(pps[idx1].x - pps[idx4].x, 2) + pow(pps[idx1].y - pps[idx4].y, 2));

                ll_px = -1;
                double m1, m2;
                if(abs(dd12 - dd13) < dd12*thr_perc){
                    ll_px = (dd12 + dd13)/2;
                    
                    m1 = atan2(pps[idx1].y - pps[idx2].y, pps[idx1].x - pps[idx2].x);
                    m2 = atan2(pps[idx1].y - pps[idx3].y, pps[idx1].x - pps[idx3].x);

                } else if(abs(dd12 - dd14) < dd12*thr_perc){
                    ll_px = (dd12 + dd14)/2;

                    m1 = atan2(pps[idx1].y - pps[idx2].y, pps[idx1].x - pps[idx2].x);
                    m2 = atan2(pps[idx1].y - pps[idx4].y, pps[idx1].x - pps[idx4].x);

                } else if(abs(dd13 - dd14) < dd13*thr_perc){
                    ll_px = (dd13 + dd14)/2;

                    m1 = atan2(pps[idx1].y - pps[idx3].y, pps[idx1].x - pps[idx3].x);
                    m2 = atan2(pps[idx1].y - pps[idx4].y, pps[idx1].x - pps[idx4].x);
                }

                if(ll_px < 0){
                    ROS_ERROR("[LANDER]: Error in Computing the Length");
                }

                // Angle Average
                mm_px = (m1 + to_PI(m2 - M_PI/2))/2;

                // Convert Angle Between -M_PI/4 <> +M_PI/4
                mm_px = mm_px > M_PI ? mm_px-M_PI : mm_px;
                mm_px = mm_px < -M_PI ? mm_px+M_PI : mm_px;
                mm_px = mm_px < 0.0 ? mm_px+(M_PI/2) : mm_px;
                mm_px = mm_px > M_PI/4 ? mm_px-(M_PI/2) : mm_px;

                return true;
            }
        }
    }

    return false;
}

void Lander::combineElements(std::vector<DistanceIdx> in, std::vector<std::vector<DistanceIdx>>& out){
    // All Combinations of 4 Elements
    for(uint i1 = 0; i1 < in.size()-3; i1++){
        for(uint i2 = i1+1; i2 < in.size()-2; i2++){
            for(uint i3 = i2+1; i3 < in.size()-1; i3++){
                for(uint i4 = i3+1; i4 < in.size(); i4++){
                    std::vector<DistanceIdx> comb;
                    
                    comb.push_back(in[i1]);
                    comb.push_back(in[i2]);
                    comb.push_back(in[i3]);
                    comb.push_back(in[i4]);

                    out.push_back(comb);
                }
            }
        }
    }
}

cv::Point Lander::findIntersection(cv::Vec2f l1, cv::Vec2f l2){
    double rho1 = l1[0];
    double theta1 = l1[1];
    double rho2 = l2[0];
    double theta2 = l2[1];

    Eigen::Matrix<double, 2, 2> A;
    Eigen::Matrix<double, 2, 1> b;
    Eigen::Matrix<double, 2, 1> x;

    A << cos(theta1), sin(theta1),
         cos(theta2), sin(theta2);
        
    b << rho1, rho2;

    x = A.colPivHouseholderQr().solve(b);

    return cv::Point(round(x(0)), round(x(1)));
}

void Lander::computeErrorSquare(std::vector<cv::Point> pps){
    // Square Centre
    scx = (pps[idx1].x + pps[idx2].x + pps[idx3].x + pps[idx4].x)/4.0;
    scy = (pps[idx1].y + pps[idx2].y + pps[idx3].y + pps[idx4].y)/4.0;

    // Camera-Square Error in Meters
    ee_xx_mm = (scx - cx)*ll_mm/ll_px;
    ee_yy_mm = (scy - cy)*ll_mm/ll_px;

    // Conversion in Body Coordinates
    double body_ee_xx = -ee_yy_mm;
    double body_ee_yy = -ee_xx_mm;

    // Conversion in Map Setpoint
    tf2::Vector3 tt(body_ee_xx, body_ee_yy, 0.0);
    tf2::Quaternion qq(0.0, 0.0, 0.0, 1.0);
    tf2::Transform tts(qq, tt);

    tf2::Transform tts_new = odomToMap_transformation*bodyToOdom_transformation*tts;
    tf2::Vector3 tt_new = tts_new.getOrigin();
    
    // Return New Setpoint
    new_xx_position = tt_new.x();
    new_yy_position = tt_new.y();
}

bool Lander::checkSquare(){
    // Check if (cx,cy) is Contained in the Square
    double ee = sqrt(pow(scx - cx, 2) + pow(scy - cy, 2));

    if(ee < sqrt(2)*ll_px/2){
        return true;
    }

    ROS_WARN_COND(debug_flag, "[LANDER]: Square Not Centered Below the Camera!");

    return false;
}

void Lander::simpleLanding(){
    // Generate a Simple Landing Trajectory
    Eigen::Matrix<double, 3, 1> init;
    Eigen::Matrix<double, 3, 1> end;

    init << actualPosition(0), actualPosition(1), actualPosition(2);
    end << actualPosition(0), actualPosition(1), end_zz;

    std::vector<Eigen::Matrix<double, 3, 1>> cp;
    cp.insert(cp.end(), 4, init);
    cp.insert(cp.end(), 4, end);

    double tt = (init-end).norm()/land_vv;

    spline->setControl(cp, tt);
}

void Lander::imageCallback(const sensor_msgs::Image::ConstPtr& msg){
	if(!haveCamInfo){
		ROS_INFO_COND(debug_flag, "[LANDER]: No Camera Info, Discarding Image");
		return;
	}

    if(processing_image){
        return;
    }

	try{
		// Read Image
        cv_bridge::CvImagePtr bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

		// Undistort Image
        cv::Mat mm_1, mm_2;
        cv::Mat ee = cv::Mat::eye(3, 3, cv::DataType<double>::type);
        cv::Size ss = {bridge->image.cols, bridge->image.rows};

        cv::fisheye::initUndistortRectifyMap(cm, dc, ee, cm, ss, CV_16SC2, mm_1, mm_2);
        cv::remap(bridge->image, bridge->image, mm_1, mm_2, cv::INTER_CUBIC, cv::BORDER_CONSTANT);

        img = bridge->image;
        image_valid = true;
    }catch(cv_bridge::Exception & e) {
        ROS_ERROR("[LANDER]: cv_bridge exception: %s", e.what());
    }
}

void Lander::infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
    if(haveCamInfo){
        return;
    }

    if(msg->K != boost::array<double, 9>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})){
        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++){
                cm.at<double>(i, j) = msg->K[i*3+j];
            }
        }

        for(int i = 0; i < 5; i++){
            dc.at<double>(0,i) = msg->D[i];
        }

        haveCamInfo = true;
    } else{
        ROS_WARN("[LANDER]: Camerainfo Message has Invalid Intrinsics");
    }
}