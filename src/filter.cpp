#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <stdio.h>


class ImageConverter
{
  // Объявляем экземпляры NodeHandle и ImageTransport
  ros::NodeHandle nh;
  image_transport::ImageTransport it;

  // Объявляем переменные, которые будут хранить имена топиков для публикации / подписки изображений
  // Они должны будут подтягиваться из параметров launch файла
  std::string sub_topic;
  std::string grayscale_pub_topic;
  std::string filtered_pub_topic;

  // Объявляем публикаторов и подписчиков
  // !!! Тут нужно объявить подписчика изображений с именем sub !!!
  // !!! Тут нужно объявить публикатора изображений с именем grayscale_pub !!!
  // !!! Тут нужно объявить публикатора изображений с именем filtered_pub !!!

  // Объявляем переменную, которая будет включать применение фильтра
  // Она должна будет подтягиваться из параметров launch файла
  bool is_processing_filter;
  
  // Объявляем ядро фильтра
  // Его значения должны будут подтягиваться из параметров launch файла
  std::vector<std::vector<float>> filter_kernel;


public:
  ImageConverter()
    : it (nh)
  {
    // Задаем размер 3х3 для ядра свертки фильтра
    filter_kernel.resize(3);
    for (int i=0; i<3; i++)
    {
      filter_kernel[i].resize(3);
    }

    // Подтягиваем все необходимые параметры из launch файла
    // !!! Тут нужно записать в переменную sub_topic значение параметра из launch файла для топика цветного изображения, на которое подписываемся !!!
    // !!! Тут нужно записать в переменную grayscale_pub_topic значение параметра из launch файла для топика серого изображения, которое будем публиковать !!!
    // !!! Тут нужно записать в переменную filtered_pub_topic значение параметра из launch файла для топика серого изображения, которое будем публиковать !!!

    // !!! Тут нужно записать в переменную is_processing_filter значение параметра из launch файла, которое отвечает за включение фильтрации !!!

    // !!! Тут нужно записать значения ядра свертки для фильтра в filter_kernel из launch файла !!!
    

    filtered_pub = it.advertise(filtered_pub_topic, 1);
    grayscale_pub = it.advertise(grayscale_pub_topic, 1);
    
    // На вход из Gazebo мы получаем цветное изображение в кодировке rgb8, то есть имеем 3 канала, на каждый из которых выделяется по 8 бит
    sub = it.subscribe(sub_topic, 1, &ImageConverter::imageCallback, this);
  }

  ~ImageConverter()
  {
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    // Объявляем переменные для серого и фильтрованного изображений формата sensor_msgs::Image
    sensor_msgs::Image grayscaled_image;
    sensor_msgs::Image filtered_image;

    // Задаем параметры изображений такие же, как у исходного цветного
    grayscaled_image.header = filtered_image.header = msg->header;
    grayscaled_image.width = filtered_image.width = msg->width;
    grayscaled_image.height = filtered_image.height = msg->height;
    grayscaled_image.is_bigendian = filtered_image.is_bigendian = false;
    grayscaled_image.step = filtered_image.step = msg->step / 3;
    grayscaled_image.data.resize(grayscaled_image.width * grayscaled_image.height);
    filtered_image.data.resize(filtered_image.width * filtered_image.height);

    // Для серого и фильтрованного изображений мы используем кодировку mono8, то есть один канал, на который выделяется 8 бит
    grayscaled_image.encoding = filtered_image.encoding = "mono8";
 
    // Небольшое вспомогательное действие, чтобы получать данные из цветного изображения в нужной кодировке
    const uint8_t* color_image_data = reinterpret_cast<const uint8_t*>(&msg->data[0]);

    // Получаем черно-белое изображение из цветного
    for (int v = 0; v < msg->height; v++)
    {
      for (int u = 0; u < msg->width; u++)
      {
        // Чтобы получить доступ к красному каналу пикселя с координатами (u,v), нужно обратиться по индексу 
        // 3*u + v*3*width, где 3 - количество каналов, в данном случае у нас их 3
        // Так как у нас цветное изображение идет в кодировке rgb8, то для каждого пикселя значение для каждого из каналов идет по порядку от красного к синему
        uint8_t red = color_image_data[3 * u + v * 3 * msg->width];
        // !!! Тут нужно получить по аналогии значения для зеленого и синего каналов пикселя в переменную !!!
        uint8_t green = 
        uint8_t blue = 
        // !!! Тут нужно посчитать значение пикселя для черно-белого изображения, используя значения RGB каналов цветного изображения
        grayscaled_image.data[u + msg->width * v] = 
        
      }
    }
    // Получаем отфильтрованное изображение из черно-белого, если значение переменной is_processing_filter стоит в true
    if (is_processing_filter)
    {
      for (int v = 0; v < grayscaled_image.height; ++v)
      {
        for (int u = 0; u < grayscaled_image.width; ++u)
        {
          float new_pixel = 0;
          
          // !!! Тут нужно записать в переменную new_pixel новое значение для каждого пикселя черно-белого изображения после операции свертки !!!
          // В данном случае так как у нас ядро свертки фильтра имеет размер 3х3, то мы добавляем для изображения границу единочного размера со значением 0
          // Это нужно для того, чтобы разрешение отфильтрованного изображения совпадало с разрешением исходного изображения

          // Записываем для каждого пикселя его новое значение после операции свертки
          filtered_image.data[u + filtered_image.width * v] = new_pixel;
          
        }
      }
      // Публикуем в топик отфильтрованное изображение
      filtered_pub.publish(filtered_image);
    }
    // Публикуем в топик черно-белое изображение
    grayscale_pub.publish(grayscaled_image);
    
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "edumip_my_robot_cv_node");
  ROS_INFO("Node for image processing was started");
  // Создаем экземпляр класса
  ImageConverter ic;
  while (ros::ok())
  {
    ros::spin();
  }
  ROS_INFO("Node for image processing was stopped");
  return 0;
}

