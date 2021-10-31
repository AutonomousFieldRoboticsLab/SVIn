/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Nov 14, 2017
 *      Author: Sharmin Rahman
 *
 *********************************************************************************/

/**
 * @file RelocFrameSynchronizer.hpp
 * @brief Header file for the RelocFrameSynchronizer class.
 * @author Sharmin Rahman
 */

#ifndef INCLUDE_OKVIS_RELOCFRAMESYNCHRONIZER_H_
#define INCLUDE_OKVIS_RELOCFRAMESYNCHRONIZER_H_

#include <atomic>
#include <mutex>
#include <condition_variable>

#include <okvis/Time.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/**
 * @brief This class is to safely notify different threads whether Reloc measurements
 *        up to a timestamp (e.g. the one of a camera frame) have already been registered.
 */
class RelocFrameSynchronizer {
 public:
  /// @brief Constructor.
  RelocFrameSynchronizer();
  /// @brief Destructor.
  ~RelocFrameSynchronizer();

  /**
   * @brief Tell the synchronizer that a new reloc measurement has been registered.
   * @param stamp Timestamp of the new reloc mewasurement.
   */
  void gotRelocData(const okvis::Time& stamp);

  /**
   * @brief Wait until a reloc measurement with a timestamp equal or newer to the supplied one is registered.
   * @param frame_stamp Timestamp until you want to have reloc measurements for.
   * @return False if a shutdown signal has been received. Otherwise true.
   */
  bool waitForUpToDateRelocData(const okvis::Time& frame_stamp);

  /// @brief Tell the synchronizer to shutdown. This will notify all waiting threads to wake up.
  void shutdown();

 private:
  okvis::Time newestRelocDataStamp_;           ///< Newest reloc data timestamp.
  okvis::Time relocDataNeededUntil_;           ///< A thread is waiting for Reloc data newer or equal to this timestamp.
  std::condition_variable gotNeededRelocData_; ///< Condition variable for waiting and notyfing.
  std::mutex mutex_;                         ///< Mutex.
  std::atomic_bool shutdown_;                ///< True if shutdown() was called.
};

} /* namespace okvis */

#endif /* INCLUDE_OKVIS_RELOCFRAMESYNCHRONIZER_H_ */
