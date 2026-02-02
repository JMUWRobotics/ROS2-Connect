// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef THREAD_HPP_
#define THREAD_HPP_

#include <atomic>
#include <thread>
#include <string>

#include <boost/core/noncopyable.hpp>

/**
 * This class is a wrapper for std::thread which implements
 * some additional callback as well as renaming the underlying
 * os-thread
 */
class Thread : boost::noncopyable {
public:
    Thread();

    virtual ~Thread();

    /**
     * Starts the thread with the given name.
     * Sets stopping to false.
     *
     * @param name the name of the thread (will be cut of after 15 chars)
     * @return true on success
     */
    bool startThread(const std::string &name);

    /**
     * Stops the thread by setting stopping to true.
     * The run-logic must react to this.
     */
    void stopThread();

    /**
     * Joins the underlying std::thread and destruct it afterwards.
     */
    void joinThread();

protected:
    std::atomic<bool> stopping;

    std::thread *thread;

    /**
     * Run method which will be executed by the underlying std::thread.
     * This method must respond to this->stopping becoming true.
     */
    virtual void run() = 0;

    /**
     * Called before the underlying thread is started.
     * This must return true for the start procedure to continue.
     * Hence this can be used to do some checks before actually starting the thread.
     *
     * @return true if thread start procedure should be continued
     */
    virtual bool onBeforeStartThread() {
        return true;
    }

    /**
     * Called right after the underlying tread was started.
     */
    virtual void onAfterStartThread() {
    }

    /**
     * Called right before the underlying thread is stopped by setting the stopping flag to true.
     */
    virtual void onBeforeStopThread() {
    }

    /**
     * Called right after the underlying thread is stopped by setting the stopping flag to true.
     */
    virtual void onStopThread() {
    }

    /**
     * Called after the underlying thread was joined.
     */
    virtual void onAfterJoinThread() {
    }

private:
    /**
     * Re-names the OS thread name by calling "pthread_setname_np".
     * The name can not exceed 16 chars incl. null termination.
     * Hence, this will cut of the given name after 15chars.
     *
     * @param name the name of the thread to be
     */
    void setThreadName(const std::string &name) const;
};

#endif /* THREAD_HPP_ */
