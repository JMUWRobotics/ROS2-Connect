// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef AUTHENTICATION_HPP
#define AUTHENTICATION_HPP

#include <boost/date_time.hpp>

#include "connect/logger.hpp"

namespace authentication {
class Authentication {
   public:
    /**
     * Retrieves the authentication information given a user key
     *
     * @param logger logger to log with
     * @param endpoint endpoint to authenticate against, should start with "/"
     * @param host host to authenticate against
     * @param port port to authenticate against
     * @param ssl if ssl should be used to authentication
     * @param userKey user key to resolve
     * @param nameSpace the namespace this is running in
     * @param domainId the domain id this is running in
     * @return true if a valid authentication information was retrieved, false otherwise
     */
    virtual bool getAuthenticationFromUserKey(const Logger &logger, const std::string &endpoint, const std::string &host, const std::string &port, const bool ssl, const std::string &userKey, const std::string &nameSpace, const int64_t domainId) = 0;

    /**
     * Sets the end of the authentication information
     *
     * @param end the end time of the authentication (special values are used to specify no end)
     */
    void setEnd(const boost::posix_time::ptime &end) {
        this->end = end;
    }

    /**
     * Sets the user principal identifying the user which was authentication
     *
     * @param user principal
     */
    void setUser(const std::string &user) {
        this->user = user;
    }

    /**
     * Returns if the authentication information is limited by an end time
     *
     * @returns if authentication information is limited by end time
     */
    bool hasEnd() const {
        return !this->end.is_special();
    }

    /**
     * Returns if this holds valid authentication information
     *
     * @returns if authentication information is valid
     */
    bool isValid() const {
        return !this->user.empty();
    }

    /**
     * Returns the end time of the authentication information.
     * Special values are used to specify no end.
     *
     * @returns end time of authentication or special value if none
     */
    const boost::posix_time::ptime &getEnd() const {
        return this->end;
    }

    /**
     * Returns the user principle
     *
     * @returns user principle
     */
    const std::string &getUser() const {
        return this->user;
    }

   protected:
    Authentication() = default;

   private:
    std::string user{""};

    boost::posix_time::ptime end{boost::posix_time::ptime(boost::posix_time::not_a_date_time)};
};
}  // namespace authentication

#endif  // AUTHENTICATION_HPP