#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <stdio.h>
#include <fstream>
#include <streambuf>

#include <curl/curl.h>

#include <boost/thread/thread.hpp>

#include <unistd.h>
#include <limits.h>
#include <ctime>

#include <openssl/sha.h>

//includes for calculating battery percentage
#include <math.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <string>

#define KEY_LOCATION "/home/bwilab/.cmasskey"
#define BASE_URL "http://nixons-head.csres.utexas.edu:7978/update?"

#define HASH_ITERATIONS 1000

#define CURL_RATE 4
#define LONG_CURL_RATE 60
#define CURL_FAIL_LIMIT 5 //after this many consecutive failures, escalade to long curl timer

//these are the constants for calculating the remaining battery life
#define A_CONST 4.85159026115e-05
#define K_CONST 0.999776391837
#define C_CONST 12.7493833452
#define EOL_VOLTAGE 10.75
double voltage = 0.0;


using namespace std;

void diagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg) {
    size_t nDiagnosticStatus = msg->status.size();
    for(size_t i = 0; i < nDiagnosticStatus; i++) {
        
        if(msg->status[i].name.compare("voltage") == 0) {
            size_t nValues = msg->status[i].values.size();
            
            for(size_t j = 0; j < nValues; j++) {
               
                if(msg->status[i].values[j].key.compare("segbot voltage: ") == 0) {
                   std::String valS = msg->status[i].values[j].value;
                   voltage = strtod(valS.c_str(), NULL);
               }
            }
        }
    }
}

//calculates the remaining battery life
double get_time_estimate(double voltage){
    double max_life = log((EOL_VOLTAGE - C_CONST) / -A) / K_CONST;
    if(voltage > C_CONST) {
        A_CONST = -A_CONST;
        return (log((EOL_VOLTAGE - C) / -A) / K + max_life);
    }
    double cur_life = log((voltage - C_CONST) / -A_CONST) / K_CONST;
    return max_life - cur_life;
}

string getSecretkey() {
  ifstream in(KEY_LOCATION, std::ios::in | std::ios::binary);
  if (in) {
    string str = std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    str.erase(std::remove(str.begin(), str.end(), '\n'), str.end());
    return str;
  }
  ROS_ERROR("Couldn't open .cmasskey");
  throw(errno);
}

string sha256(const string str) {
    unsigned char hash[SHA256_DIGEST_LENGTH];
    SHA256_CTX sha256;
    SHA256_Init(&sha256);
    SHA256_Update(&sha256, str.c_str(), str.size());
    SHA256_Final(hash, &sha256);
    stringstream ss;
    for(int i = 0; i < SHA256_DIGEST_LENGTH; i++) {
        ss << hex << setw(2) << setfill('0') << (int)hash[i];
    }
    return ss.str();
}

string hash(string msg, string salt) {
    for (int i = 0; i < HASH_ITERATIONS; i++) {
        msg = sha256(msg + salt);
    }
    return msg;
}

float x;
float y;
boost::shared_ptr<ros::Subscriber> location_subscriber_;

void locationHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose) {
  x = pose->pose.pose.position.x;
  y = pose->pose.pose.position.y;
}

int main(int argc, char **argv){

  //declare our node to ROS
  ros::init(argc, argv, "cmass_client");
  ros::NodeHandle n;

  location_subscriber_.reset(new ros::Subscriber);
  *location_subscriber_ = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1, &locationHandler);
    
  //subcriber to robot battery voltage
  ros::Subscriber sub = n.subscribe("diagnostics", 1000, diagnosticsCallback);      
    
  // retrieve the computer and user names
  char hostname[HOST_NAME_MAX];
  char username[LOGIN_NAME_MAX];
  gethostname(hostname, HOST_NAME_MAX);
  getlogin_r(username, LOGIN_NAME_MAX);

  CURL *curl;
  CURLcode res;
  curl = curl_easy_init();
  ros::Rate curl_rate(1.0f / CURL_RATE); // rate takes frequency, so take reciprocal
  ros::Rate slow_curl_rate( 1.0f / LONG_CURL_RATE);
  ros::Rate current_rate = curl_rate;

  int consecutive_curl_failures = 0;

  while(ros::ok()) {
    ros::spinOnce();

    //send http request
    if(curl) {

        
      std::string name_str = "name=" + boost::lexical_cast<std::string>(hostname);
      std::string timestamp_str = "timestamp=" + boost::lexical_cast<std::string>(time(0));
      std::string user_str = "user=" + boost::lexical_cast<std::string>(username);
      std::string x_str = "x=" + boost::lexical_cast<std::string>(x);
      std::string y_str = "y=" + boost::lexical_cast<std::string>(y);
      std::string time_remaining_str = "time_remaining=" + boost::lexical_cast<std::string>(get_estimate(voltage));//change params

      // IMPORTANT: URL params must be in alphabetical order by key (except for token)
      // because of the way that the server decodes them.
      std::string params = name_str + "&" + timestamp_str + "&" + user_str + "&" + x_str + "&" + y_str + "&" + time_remaining_str;
      std::string token = hash(params, getSecretkey());
      std::string token_str = "token=" + boost::lexical_cast<std::string>(token);

      std::string url = BASE_URL + params + "&" + token_str;
      ROS_INFO("url: %s", url.c_str());

      //perform the curl
      curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
      curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
      res = curl_easy_perform(curl);
      if(res != CURLE_OK) { // if the curl fails
        ROS_ERROR("curl operation failed: %s",  curl_easy_strerror(res));
        if (consecutive_curl_failures >= CURL_FAIL_LIMIT) {
          ROS_ERROR("consecutive curl failure limit reached, escalading to long curl timer");
          current_rate = slow_curl_rate;
        }
        consecutive_curl_failures++;
      }

      else {
        consecutive_curl_failures = 0;
        current_rate = curl_rate;
      }
    }
    curl_rate.sleep();
  }

  curl_easy_cleanup(curl);
  return 0;
}
