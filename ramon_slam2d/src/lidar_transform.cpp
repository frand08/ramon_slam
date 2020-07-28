#include <ros/ros.h>

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "nav_msgs/OccupancyGrid.h"

#include "contour_extraction.h"
#include "brasenham_line_algorithm.h"
#include "mle.h"

// #define SEPARATE_POINTS         // Para solo tomar dos datos a una distancia de tiempo dado
#define REGEN_MAP               // Para que se regenere o no el mapa con cada nuevo set de datos
// #define GET_RIGIDBODY_TRANSFORM // Para calcular la transformada entre el nuevo set y el mapa

ros::Publisher map_pub;
ros::Publisher pose_pub;

#ifdef SEPARATE_POINTS
int count = 0, processing = 1;
#endif

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensor_msgs::LaserScan scan = (sensor_msgs::LaserScan)*msg;
    // uint32_t lidar_range_readings = uint32_t(((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1);
    static std::vector<float> density; 
    static bool first_time = true;
    static uint32_t seq = 0;
    static std::vector<sensor_msgs::PointCloud> contours;
    static int data_count = 0;
    float map_resolution = 0.05;
    float map_update_rate = 5.0;
    uint32_t map_width = 20;
    uint32_t map_height = 20;

    static nav_msgs::OccupancyGrid lidar_map;
    static std::vector< std::vector<geometry_msgs::Point32> > contour_array;

    // Uno guarda puntos y el otro se basa en estos puntos para guardar los contours de ellos
    std::vector<geometry_msgs::Point32> contour_aux,points32_aux;

#ifdef GET_RIGIDBODY_TRANSFORM 
    std::vector<float> values_likelihood;
    float scan_likelihood = 0;
#endif

    static geometry_msgs::PoseStamped pose;

    // Para calcular tiempos del algoritmo
    ros::WallTime start_, end_;

    std::vector<geometry_msgs::Point32> points_free;

    float dis_threshold = 0.1;
    int i, j;

    // Para LS
    float x_ls = 0;

    // m squares
    float footprint = 0.1;

    uint32_t map_point = 0;
    uint32_t pointCount = 0;

    float std_dev = 0.1;    // Valor fruta

    bool map_regen;     // Para setear si se regenera siempre el mapa o si no

    // Para el mle de cada punto
    std::vector<int8_t> likelihood;

#ifdef REGEN_MAP
    map_regen = true;
#else
    map_regen = false;
#endif

    start_ = ros::WallTime::now();

    // Para inicializar todo la primera vez que entro
    if(first_time)
    {
        // Cargamos los ids de los frames de cada uno
        lidar_map.header.frame_id = "map";
        pose.header.frame_id= "pose";

        // Init del mapa
        lidar_map.info.height = map_height/map_resolution;
        lidar_map.info.width = map_width/map_resolution;
        lidar_map.info.resolution = map_resolution;
        lidar_map.info.origin.position.x = 0;
        lidar_map.info.origin.position.y = 0;
        lidar_map.info.origin.position.z = 0;
        for(i = 0; i < (map_width*map_height)/(map_resolution*map_resolution); i++)
            lidar_map.data.push_back(-1);

        // Para el pose
        pose.pose.position.x = double(map_height/2);
        pose.pose.position.y = double(map_width/2);
        pose.pose.position.z = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;

        ROS_INFO("Size del mapa:%zd",lidar_map.data.size());
        // ROS_INFO("Lecturas tomadas por vuelta: %u",lidar_range_readings);
    }
    
#ifdef SEPARATE_POINTS
    if(!processing || count >= 2 || (data_count < 31 && data_count > 0))
    {
        data_count++;
        return;
    }
    else
    {
        data_count = 0;
        processing = 0;
    }
#endif

#ifdef REGEN_MAP
    for(i = 0; i < (map_width*map_height)/(map_resolution*map_resolution); i++)
    {
        lidar_map.data[i] = -1;
        contour_array.clear();
    }
#endif

    // Primero, obtengo todos los puntos de interes
    for (float angle = scan.angle_min; angle < scan.angle_max; angle += scan.angle_increment)
    {
        geometry_msgs::Point32 point;

        if(scan.ranges[pointCount] > 0.005 && scan.ranges[pointCount] < scan.range_max)
        {
            float dis = scan.ranges[pointCount];
            point.x = -(sin(angle) * dis);
            point.y = cos(angle) * dis;
            point.z = 0;

            points32_aux.push_back(point);

// Creo que meti fruta fuerte aca, no lo entendi
#ifdef GET_RIGIDBODY_TRANSFORM 
            //Calculo distancias en caso de que no sea la primera vez
            if(!first_time)
            {
                // Para LS de la exponencial
                std::vector<float> a, b;
                float a_sum = 0, b_sum = 0;

                float min_dist;
                uint32_t min_dist_index[2];
                
                for(i = 0; i < contour_array.size(); i++)
                {
                    for(j = 0; j < contour_array[i].size(); j++)
                    {
                        float x_value = contour_array[i][j].x/lidar_map.info.resolution;
                        float y_value = contour_array[i][j].y/lidar_map.info.resolution;
                        float min_dist_aux = sqrt(pow(x_value,2)+pow(y_value,2));
                        if(j == 0 && i == 0)
                        {
                            min_dist = min_dist_aux;
                            min_dist_index[0] = 0;
                            min_dist_index[1] = 0;
                        }
                        else
                        {
                            if(min_dist_aux < min_dist)
                            {
                                min_dist = min_dist_aux;
                                min_dist_index[0] = i;
                                min_dist_index[1] = j;
                            }
                        }
                    }
                }

                values_likelihood.push_back(get_likelihood(min_dist, 0, std_dev));
                scan_likelihood += get_likelihood(min_dist, 0, std_dev);

/*
                // LS Fitting exponential:
                // http://mathworld.wolfram.com/LeastSquaresFittingExponential.html
                // a: en realidad es 0 para el caso
                float x = min_dist, y = values_likelihood[pointCount];
                a.push_back(((x*x*y)*(y*log(y)) - (x*y)*(x*y*log(y))) / (y*(x*x*y) - (x*y)*(x*y)));
                b.push_back((y*(x*y*log(y)) - (x*y)*(y*log(y))) / (y*(x*x*y) - (x*y)*(x*y)));
                a_sum += a[a.size()-1]; 
                b_sum += b[b.size()-1];
                x_ls += (y*(log(y) - b[b.size()-1]*x))*(y*(log(y) - b[b.size()-1]*x));
*/
            }
#else

#endif
        }
        pointCount++;
    }

    // Luego, extraigo los contours para la primera vez en caso de que no tenga que
    // regenerar el mapa continuamente
    // Obtiene contours, de una forma muy rudimentaria, habria que mejorarlo
    contour_extraction(contour_array, points32_aux, dis_threshold);
    ROS_INFO("Contours realizados, con:\n\taux:%zd\n\tarray:%zd",contour_aux.size(),contour_array.size());

    // if(first_time || map_regen == true)
    // {

        points_free.clear();

        for(i=0; i<contour_array.size(); i++)
        {
            for(j=0; j<contour_array[i].size(); j++)
            {
                if(first_time || map_regen == true)
                {
                
                    float x_value = contour_array[i][j].x/lidar_map.info.resolution;
                    float y_value = contour_array[i][j].y/lidar_map.info.resolution;
                    
                    squares_line(0, 0, x_value, y_value, points_free);

                    if(points_free.size() > 0)
                    {
                        for(int mongo = 0; mongo < points_free.size(); mongo++)
                        {
                            map_point = lidar_map.info.height*lidar_map.info.width/2 - lidar_map.info.width/2 + 
                                        int(int(points_free[mongo].y)*lidar_map.info.height + 
                                        int(points_free[mongo].x));

                            if(lidar_map.data[map_point] == -1)
                            {
                                lidar_map.data[map_point] = 0;
                            }
                        }
                        points_free.clear();
                    }
                    // Obtengo el punto al que equivale el dato en el mapa
                    map_point = lidar_map.info.height*lidar_map.info.width/2 - lidar_map.info.width/2 + 
                                int(int(contour_array[i][j].y/lidar_map.info.resolution)*lidar_map.info.height + 
                                int(contour_array[i][j].x/lidar_map.info.resolution));
                    lidar_map.data[map_point] = 100;
                }
                else
                {
                    // Obtengo el punto al que equivale el dato en el mapa
                    map_point = lidar_map.info.height*lidar_map.info.width/2 - lidar_map.info.width/2 + 
                                int(int(contour_array[i][j].y/lidar_map.info.resolution)*lidar_map.info.height + 
                                int(contour_array[i][j].x/lidar_map.info.resolution));

                    // // Cuento si cae en 100 o si no
                    // if(lidar_map.data[map_point] >= 0)
                    //     count_points += lidar_map.data[map_point];                

                }
                                
                /*
                get_occupancy_likelihood(likelihood,contour_array[i][j],map_resolution,std_dev,std_dev);

                // [0][0]
                int point_aux = map_point - lidar_map.info.height - 1;
                if(lidar_map.data[point_aux] > likelihood[0] || 
                  lidar_map.data[point_aux] < 0)
                {
                    lidar_map.data[point_aux] = likelihood[0];
                }

                // [0][1]
                point_aux = map_point - lidar_map.info.height;
                if(lidar_map.data[point_aux] > likelihood[1] || 
                   lidar_map.data[point_aux] < 0) 
                {
                    lidar_map.data[point_aux] = likelihood[1];
                }
                
                // [0][2]
                point_aux = map_point - lidar_map.info.height + 1;
                if(lidar_map.data[point_aux] > likelihood[2] || 
                  lidar_map.data[point_aux] < 0) 
                {
                    lidar_map.data[point_aux] = likelihood[2];
                }

                // [1][0]
                point_aux = map_point - 1;
                if(lidar_map.data[point_aux] > likelihood[3] || 
                  lidar_map.data[point_aux] < 0) 
                {
                    lidar_map.data[point_aux] = likelihood[3];
                }
                
                // [1][1]
                point_aux = map_point;
                if(lidar_map.data[point_aux] > likelihood[4] || 
                  lidar_map.data[point_aux] < 0) 
                {
                    lidar_map.data[point_aux] = likelihood[4];
                }

                // [1][2]
                point_aux = map_point + 1;
                if(lidar_map.data[point_aux] > likelihood[5] || 
                  lidar_map.data[point_aux] < 0)
                {
                    lidar_map.data[point_aux] = likelihood[5];
                }

                // [2][0]
                point_aux = map_point + lidar_map.info.height - 1;
                if(lidar_map.data[point_aux] > likelihood[6] || 
                  lidar_map.data[point_aux] < 0)
                {
                    lidar_map.data[point_aux] = likelihood[6];
                }

                // [2][1]
                point_aux = map_point + lidar_map.info.height;
                if(lidar_map.data[point_aux] > likelihood[7] || 
                  lidar_map.data[point_aux] < 0)
                {
                    lidar_map.data[point_aux] = likelihood[7];
                }

                // [2][2]
                point_aux = map_point + lidar_map.info.height + 1;
                if(lidar_map.data[point_aux] > likelihood[8] || 
                  lidar_map.data[point_aux] < 0)
                {
                    lidar_map.data[point_aux] = likelihood[8];
                }
                */
            }        
        }
    // }
    
    end_ = ros::WallTime::now();
    double execution_time = (end_ - start_).toNSec() * 1e-6;
    ROS_INFO_STREAM("Exectution time (ms): " << execution_time);

    lidar_map.header.stamp = ros::Time();
    pose.header.stamp = ros::Time();

    first_time = false;

    map_pub.publish(lidar_map);
    pose_pub.publish(pose);

#ifdef SEPARATE_POINTS
    count++;
    data_count++;
    ROS_INFO("Cantidad de veces ejecutadas: %d",count);
#endif
}


int main (int argc, char **argv)
{
	ros::init(argc, argv, "lidar_transform");
	ros::NodeHandle n;
	ros::Subscriber scanSubscriber = n.subscribe("scan", 1000, scanCallback);

    pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose",10);
    map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1000);    

    // ros::spin();

    ros::Rate loop_rate(100.0);
    // ros::Duration loop_duration(15.0);
    while (ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
        // loop_duration.sleep();
#ifdef SEPARATE_POINTS
        processing = 1;
#endif
    }
	
	return 0;
}