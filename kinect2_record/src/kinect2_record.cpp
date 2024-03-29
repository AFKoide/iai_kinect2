#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <chrono>

class ImageSubscriberNode {
public:
    ImageSubscriberNode() : nh("~") {
        // Initialisation des paramètres
        nh.param<std::string>("topic", topic_name, "/kinect2/hd/image_color_rect");
        nh.param<double>("duration", duration, 30.0);

        // Abonnement au topic
        image_sub = nh.subscribe(topic_name, 1, &ImageSubscriberNode::imageCallback, this);

        // Démarrage du chrono
        start_time = std::chrono::steady_clock::now();
    }

    void run() {
        ros::Rate rate(10); // Fréquence de vérification des événements ROS
        while (ros::ok()) {
            ros::spinOnce();

            // Vérifier si le temps écoulé dépasse la durée prédéterminée
            auto current_time = std::chrono::steady_clock::now();
            double elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
            if (elapsed_time >= duration) {
                ROS_INFO("Temps écoulé. Fermeture de la node.");
                break;
            }

            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber image_sub;
    std::string topic_name;
    double duration;
    std::chrono::steady_clock::time_point start_time;

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        // Convertir l'image au format OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Enregistrer l'image dans le dossier actuel
        std::string file_name = "image_" + std::to_string(std::time(nullptr)) + ".jpg";
        cv::imwrite(file_name, cv_ptr->image);
        ROS_INFO("Image enregistrée sous : %s", file_name.c_str());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_subscriber_node");
    ImageSubscriberNode node;
    node.run();
    return 0;
}
