/**
 * RedisClient.cpp
 *
 * Author: Toki Migimatsu
 * Created: April 2017
 */

#include "RedisClient.h"
#include <iostream>
#include <sstream>
#include <cstring>
#include <tuple>


void RedisClient::connect(const std::string& hostname, const int port, const struct timeval& timeout) {
    // Connect to new server
    context_.reset(nullptr);
    redisContext *c= redisConnectWithTimeout(hostname.c_str(), port, timeout);
    std::unique_ptr<redisContext, redisContextDeleter> context(c);

    // Check for errors
    if (!context)
        throw std::runtime_error("RedisClient: Could not allocate redis context.");
    if (context->err)
        throw std::runtime_error("RedisClient: Could not connect to redis server: " + std::string(context->errstr));

    // Save context
    context_ = std::move(context);
}

std::unique_ptr<redisReply, redisReplyDeleter> RedisClient::command(const char *format, ...) {
    va_list ap;
    va_start(ap, format);
    redisReply *reply = (redisReply *)redisvCommand(context_.get(), format, ap);
    va_end(ap);
    return std::unique_ptr<redisReply, redisReplyDeleter>(reply);
}

void RedisClient::ping() {
    auto reply = command("PING");
    std::cout << std::endl << "RedisClient: PING " << context_->tcp.host << ":" << context_->tcp.port << std::endl;
    if (!reply) throw std::runtime_error("RedisClient: PING failed.");
    std::cout << "Reply: " << reply->str << std::endl << std::endl;
}

std::string RedisClient::get(const std::string& key) {
    // Call GET command
    auto reply = command("GET %s", key.c_str());

    // Check for errors
    if (!reply || reply->type == REDIS_REPLY_ERROR || reply->type == REDIS_REPLY_NIL)
        throw std::runtime_error("RedisClient: GET '" + key + "' failed.");
    if (reply->type != REDIS_REPLY_STRING)
        throw std::runtime_error("RedisClient: GET '" + key + "' returned non-string value.");

    // Return value
    return reply->str;
}

void RedisClient::set(const std::string& key, const std::string& value) {
    // Call SET command
    auto reply = command("SET %s %s", key.c_str(), value.c_str());

    // Check for errors
    if (!reply || reply->type == REDIS_REPLY_ERROR)
        throw std::runtime_error("RedisClient: SET '" + key + "' '" + value + "' failed.");
}

void RedisClient::del(const std::string& key) {
    // Call DEL command
    auto reply = command("DEL %s", key.c_str());

    // Check for errors
    if (!reply || reply->type == REDIS_REPLY_ERROR)
        throw std::runtime_error("RedisClient: DEL '" + key + "' failed.");
}

std::vector<std::string> RedisClient::pipeget(const std::vector<std::string>& keys) {
    // Prepare key list
    for (const auto& key : keys) {
        redisAppendCommand(context_.get(), "GET %s", key.c_str());
    }

    // Collect values
    std::vector<std::string> values;
    for (size_t i = 0; i < keys.size(); i++) {
        redisReply *r;
        if (redisGetReply(context_.get(), (void **)&r) == REDIS_ERR)
            throw std::runtime_error("RedisClient: Pipeline GET command failed for key:" + keys[i] + ".");

        std::unique_ptr<redisReply, redisReplyDeleter> reply(r);
        if (reply->type != REDIS_REPLY_STRING)
            throw std::runtime_error("RedisClient: Pipeline GET command returned non-string value for key: " + keys[i] + ".");
        values.push_back(reply->str);
    }
    return values;
}

void RedisClient::pipeset(const std::vector<std::pair<std::string, std::string>>& keyvals) {
    // Prepare key list
    for (const auto& keyval : keyvals) {
        redisAppendCommand(context_.get(), "SET %s %s", keyval.first.c_str(), keyval.second.c_str());
    }

    for (size_t i = 0; i < keyvals.size(); i++) {
        redisReply *r;
        if (redisGetReply(context_.get(), (void **)&r) == REDIS_ERR)
            throw std::runtime_error("RedisClient: Pipeline SET command failed for key: " + keyvals[i].first + ".");

        std::unique_ptr<redisReply, redisReplyDeleter> reply(r);
        if (reply->type == REDIS_REPLY_ERROR)
            throw std::runtime_error("RedisClient: Pipeline SET command failed for key: " + keyvals[i].first + ".");
    }
}

std::vector<std::string> RedisClient::mget(const std::vector<std::string>& keys) {
    // Prepare key list
    std::vector<const char *> argv = {"MGET"};
    for (const auto& key : keys) {
        argv.push_back(key.c_str());
    }

    // Call MGET command with variable argument formatting
    redisReply *r = (redisReply *)redisCommandArgv(context_.get(), argv.size(), &argv[0], nullptr);
    std::unique_ptr<redisReply, redisReplyDeleter> reply(r);

    // Check for errors
    if (!reply || reply->type != REDIS_REPLY_ARRAY)
        throw std::runtime_error("RedisClient: MGET command failed.");

    // Collect values
    std::vector<std::string> values;
    for (size_t i = 0; i < reply->elements; i++) {
        values.push_back(reply->element[i]->str);
    }
    return values;
}

void RedisClient::mset(const std::vector<std::pair<std::string, std::string>>& keyvals) {
    // Prepare key-value list
    std::vector<const char *> argv = {"MSET"};
    for (const auto& keyval : keyvals) {
        argv.push_back(keyval.first.c_str());
        argv.push_back(keyval.second.c_str());
    }

    // Call MSET command with variable argument formatting
    redisReply *r = (redisReply *)redisCommandArgv(context_.get(), argv.size(), &argv[0], nullptr);
    std::unique_ptr<redisReply, redisReplyDeleter> reply(r);

    // Check for errors
    if (!reply || reply->type == REDIS_REPLY_ERROR)
        throw std::runtime_error("RedisClient: MSET command failed.");
}


