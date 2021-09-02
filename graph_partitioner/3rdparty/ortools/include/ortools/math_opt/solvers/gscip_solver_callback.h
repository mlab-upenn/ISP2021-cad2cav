// Copyright 2010-2021 Google LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OR_TOOLS_MATH_OPT_SOLVERS_GSCIP_SOLVER_CALLBACK_H_
#define OR_TOOLS_MATH_OPT_SOLVERS_GSCIP_SOLVER_CALLBACK_H_

#include <memory>
#include <optional>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "scip/type_scip.h"
#include "ortools/gscip/gscip.h"
#include "ortools/math_opt/callback.pb.h"
#include "ortools/math_opt/solver_interface.h"
#include "ortools/math_opt/solvers/message_callback_data.h"

namespace operations_research {
namespace math_opt {

// Handler for user callbacks for GScipSolver.
//
// It deals with solve interruption when the user request it or when an error
// occurs during the call of the user callback. Any such error is returned by
// Flush().
class GScipSolverCallbackHandler {
 public:
  // Returns a non null handler if needed (there are supported events that we
  // register to).
  //
  // The caller will also have to use MessageHandler() when calling
  // GScip::Solve() when the result is not nullptr.
  //
  // At the end of the solve, Flush() must be called (when everything else
  // succeeded) to make the final user callback calls and return the first error
  // that occurred when calling the user callback.
  static std::unique_ptr<GScipSolverCallbackHandler> RegisterIfNeeded(
      const CallbackRegistrationProto& callback_registration,
      SolverInterface::Callback callback, absl::Time solve_start, SCIP* scip);

  GScipSolverCallbackHandler(const GScipSolverCallbackHandler&) = delete;
  GScipSolverCallbackHandler& operator=(const GScipSolverCallbackHandler&) =
      delete;

  // Returns the handler to pass to GScip::Solve().
  GScipMessageHandler MessageHandler();

  // Makes any last pending calls and returns the first error that occurred
  // while calling the user callback. Returns OkStatus if no error has occurred.
  absl::Status Flush();

 private:
  GScipSolverCallbackHandler(SolverInterface::Callback callback,
                             absl::Time solve_start, SCIP* scip);

  // Updates message_callback_data_ and makes the necessary calls to the user
  // callback if necessary. This method has the expected signature for a
  // GScipMessageHandler.
  void MessageCallback(GScipMessageType type, absl::string_view message);

  // Makes a call to the user callback, updating the status_ and interrupting
  // the solve if needed (in case of error or if requested by the user).
  //
  // This function will ignores calls when status_ is not ok. It returns the
  // result of the call of the callback when the call has successfully been made
  // and the user has not requested the termination of the solve.
  //
  // This function will hold the callback_mutex_ while making the call to the
  // user callback to serialize calls.
  absl::optional<CallbackResultProto> CallUserCallback(
      const CallbackDataProto& callback_data)
      ABSL_LOCKS_EXCLUDED(callback_mutex_);

  // The user callback. Should be called via CallUserCallback().
  const SolverInterface::Callback callback_;

  // Start time of the solve.
  const absl::Time solve_start_;

  // The SCIP solver, used for interruptions.
  SCIP* const scip_;

  // Mutex serializing calls to the user callback and the access to status_.
  absl::Mutex callback_mutex_;

  // The first error status returned by the user callback.
  absl::Status status_ ABSL_GUARDED_BY(callback_mutex_);

  // Mutex serializing access to message_callback_data_ and the serialization of
  // calls to the user callback for CALLBACK_EVENT_MESSAGE events.
  absl::Mutex message_mutex_;

  // The buffer used to generate the message events.
  MessageCallbackData message_callback_data_ ABSL_GUARDED_BY(message_mutex_);
};

}  // namespace math_opt
}  // namespace operations_research

#endif  // OR_TOOLS_MATH_OPT_SOLVERS_GSCIP_SOLVER_CALLBACK_H_
