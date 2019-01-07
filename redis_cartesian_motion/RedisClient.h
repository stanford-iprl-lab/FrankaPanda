/**
 * RedisClient.h
 *
 * Author: Toki Migimatsu
 *         Shameek Ganguly
 * Created: April 2017
 */

#ifndef REDIS_CLIENT_H
#define REDIS_CLIENT_H

#include <hiredis/hiredis.h>
#include <string>
#include <cstring>
#include <iostream>
#include <sstream>
#include <vector>
#include <tuple>
#include <thread>
#include <chrono>
#include <stdexcept>

namespace RedisServer {
    // Default server ip
    const std::string DEFAULT_IP = "127.0.0.1";

    // Default server port
    const int DEFAULT_PORT = 6379;

    // Default Redis key prefix
    static const std::string KEY_PREFIX = "Kinect::";
}

struct redisReplyDeleter {
    void operator()(redisReply *r) { freeReplyObject(r); }
};
struct redisContextDeleter {
    void operator()(redisContext *c) { redisFree(c); }
};


class RedisClient {

public:
    std::unique_ptr<redisContext, redisContextDeleter> context_;

    /**
     * Connect to Redis server.
     *
     * @param hostname  Redis server IP address (default 127.0.0.1).
     * @param port      Redis server port number (default 6379).
     * @param timeout   Connection attempt timeout (default 1.5s).
     */
    void connect(const std::string& hostname="127.0.0.1", const int port=6379, const struct timeval& timeout= {1, 500000} );

    /**
     * Issue a command to Redis.
     *
     * This function is a C++ wrapper around hiredis::redisCommand() that
     * provides a self-freeing redisReply pointer. The command is formatted in
     * printf() style.
     *
     * @param format  Format string.
     * @param ...     Format values.
     * @return        redisReply pointer.
     */
    std::unique_ptr<redisReply, redisReplyDeleter> command(const char *format, ...);

    /**
     * Perform Redis command: PING.
     *
     * If the server is responsive, it should reply PONG.
     */
    void ping();

    /**
     * Perform Redis command: GET key.
     *
     * @param key  Key to get from Redis (entry must be String type).
     * @return     Retrieved string value.
     */
    std::string get(const std::string& key);

    /**
     * Perform Redis command: SET key value.
     *
     * @param key    Key to set in Redis.
     * @param value  Value for key.
     */
    void set(const std::string& key, const std::string& value);

    /**
     * Perform Redis command: DEL key.
     *
     * @param key    Key to delete in Redis.
     */
    void del(const std::string& key);

    /**
     * Perform Redis GET commands in bulk: GET key1; GET key2...
     *
     * Pipeget gets multiple keys as a non-atomic operation. More efficient than
     * getting the keys separately. See:
     * https://redis.io/topics/mass-insert
     *
     * In C++11, this function can be called with brace initialization:
     * auto values = redis_client.pipeget({"key1", "key2"});
     * 
     * @param keys  Vector of keys to get from Redis.
     * @return      Vector of retrieved values. Optimized with RVO.
     */
    std::vector<std::string> pipeget(const std::vector<std::string>& keys);

    /**
     * Perform Redis SET commands in bulk: SET key1 val1; SET key2 val2...
     *
     * Pipeset sets multiple keys as a non-atomic operation. More efficient than
     * setting the keys separately. See:
     * https://redis.io/topics/mass-insert
     *
     * In C++11, this function can be called with brace initialization:
     * redis_client.pipeset({{"key1", "val1"}, {"key2", "val2"}});
     *
     * @param keyvals  Vector of key-value pairs to set in Redis.
     */
    void pipeset(const std::vector<std::pair<std::string, std::string> >& keyvals);

    /**
     * Perform Redis command: MGET key1 key2...
     *
     * MGET gets multiple keys as an atomic operation. See:
     * https://redis.io/commands/mget
     *
     * @param keys  Vector of keys to get from Redis.
     * @return      Vector of retrieved values. Optimized with RVO.
     */
    std::vector<std::string> mget(const std::vector<std::string>& keys);

    /**
     * Perform Redis command: MSET key1 val1 key2 val2...
     *
     * MSET sets multiple keys as an atomic operation. See:
     * https://redis.io/commands/mset
     *
     * @param keyvals  Vector of key-value pairs to set in Redis.
     */
    void mset(const std::vector<std::pair<std::string, std::string> >& keyvals);

    template<long unsigned int n>
    bool hDoubleArraytoStringJSON(const std::array<double, n> &x, const int& array_length, std::string& arg_str) {
      std::stringstream ss;
      ss << "[";
      for(int i=0; i<array_length; ++i) {
        if(i>0) ss << ",";
        ss<<x[i];
      }
      ss << "]";
      arg_str = ss.str();
      return true;
    }

    template<long unsigned int n>
    bool hDoubleArrayfromStringJSON(std::array<double, n> &x, const int& array_length, const std::string& arg_str) {
      if (arg_str.front() != '[') {
        std::cout << "string does not start with [" << std::endl;
        return false;
      }
      if (arg_str.back() != ']') {
        std::cout << "string does not end with ]" << std::endl;
        return false;
      }
      std::string substring = arg_str.substr(1, arg_str.length()-2);
      char * c_substring = new char[substring.size()+1];
      std::strcpy(c_substring, substring.c_str());
      int i = 0;
      char * val;
      val = strtok (c_substring, ",");
      while (val != NULL)
      {
        x[i] = atof(val);
        val = strtok (NULL, ",");
        i++;
        if (i > array_length) {
          std::cout << "string contains more numbers than specified" << std::endl;
        }
      }
      delete [] c_substring;
      if (i < array_length) {
        std::cout << "string contains less numbers than specified" << std::endl;
        return false;
      }
      return true;
    }

};


#endif //REDIS_CLIENT_H
