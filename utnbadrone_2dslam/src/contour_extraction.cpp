#include <contour_extraction.h>

void contour_extraction(std::vector<std::vector<geometry_msgs::Point32> > &contour_array,
                        std::vector<geometry_msgs::Point32> &points32_aux,
                        float dis_threshold)
{
    int i;
    std::vector<geometry_msgs::Point32> contour_aux;
    float pointmod_reg, pointmod_next, pointmod_res;

    for(i = 1; i < points32_aux.size(); i++)
    {
        pointmod_reg = sqrt(pow(points32_aux[i-1].x,2)+pow(points32_aux[i-1].y,2));
        pointmod_next = sqrt(pow(points32_aux[i].x,2)+pow(points32_aux[i].y,2));

        if(pointmod_reg >= pointmod_next)
            pointmod_res = pointmod_reg - pointmod_next;
        else
            pointmod_res = pointmod_next - pointmod_reg;
        
        // Si el resto del modulo de los dos puntos contiguos es menor que el umbral, entonces
        // pueden formar parte de un contorno
        if(pointmod_res < dis_threshold)
        {
            if(contour_aux.size() == 0)
            {
                contour_aux.push_back(points32_aux[i-1]);
            }
            contour_aux.push_back(points32_aux[i]);
        }
        else
        {
            // Si tengo por lo menos X puntos contiguos, puedo decir que es un contorno
            if(contour_aux.size() > 5)
            {
                // Lo agrego al array de contornos
                contour_array.push_back(contour_aux);
            }
            contour_aux.clear();
        }
    }

    // Si tengo todavia quiere decir que seguia, en principio, el contour y termino por llegar
    // al maximo numero
    if(contour_aux.size() > 2 && contour_array.size() > 0)
    {
        // Tomo el ultimo punto del ultimo point auxiliar tomado
        pointmod_reg = sqrt(pow(contour_aux[contour_aux.size()-1].x,2) + 
                            pow(contour_aux[contour_aux.size()-1].y,2));

        // Y lo comparo con el primero te todos los puntos tomados
        pointmod_next = sqrt(pow(contour_array[0][0].x,2) + 
                             pow(contour_array[0][0].y,2));
        
        // ROS_INFO("Modulos realizados");
        
        if(pointmod_reg >= pointmod_next)
            pointmod_res = pointmod_reg - pointmod_next;
        else
            pointmod_res = pointmod_next - pointmod_reg;

        // En caso de que de tenga una distancia menor que la umbral (el modulo), 
        // lo uno con el primero
        if(pointmod_res < dis_threshold)
        {
            // ROS_INFO("Por unir principio con fin");

            for(i=0; i<contour_array[0].size(); i++)
                contour_aux.push_back(contour_array[0][i]);

            // Borro primer entrada
            contour_array.erase(contour_array.begin());

            // Y la cargo de vuelta en la primera posicion
            contour_array.insert(contour_array.begin(),contour_aux);
        }

        else if(contour_aux.size() > 5)
        {
            contour_array.push_back(contour_aux);
        }

        contour_aux.clear();
    }

}