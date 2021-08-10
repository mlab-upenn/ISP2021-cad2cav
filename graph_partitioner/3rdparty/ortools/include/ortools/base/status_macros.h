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

#ifndef OR_TOOLS_BASE_STATUS_MACROS_H_
#define OR_TOOLS_BASE_STATUS_MACROS_H_

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "ortools/base/status_builder.h"

namespace absl {

// Run a command that returns a absl::Status.  If the called code returns an
// error status, return that status up out of this method too.
//
// Example:
//   RETURN_IF_ERROR(DoThings(4));
//   RETURN_IF_ERROR(DoThings(5)) << "Additional error context";
#define RETURN_IF_ERROR(expr)                                \
  switch (0)                                                 \
  case 0:                                                    \
  default:                                                   \
    if (const ::absl::Status status = (expr); status.ok()) { \
    } else /* NOLINT */                                      \
      return ::util::StatusBuilder(status)

// Internal helper for concatenating macro values.
#define STATUS_MACROS_CONCAT_NAME_INNER(x, y) x##y
#define STATUS_MACROS_CONCAT_NAME(x, y) STATUS_MACROS_CONCAT_NAME_INNER(x, y)

#define ASSIGN_OR_RETURN_IMPL(statusor, lhs, rexpr) \
  auto statusor = (rexpr);                          \
  RETURN_IF_ERROR(statusor.status());               \
  lhs = *std::move(statusor)

// Executes an expression that returns an absl::StatusOr, extracting its value
// into the variable defined by lhs (or returning on error).
//
// Example: Assigning to an existing value
//   ValueType value;
//   ASSIGN_OR_RETURN(value, MaybeGetValue(arg));
//
// WARNING: ASSIGN_OR_RETURN expands into multiple statements; it cannot be used
//  in a single statement (e.g. as the body of an if statement without {})!
#define ASSIGN_OR_RETURN(lhs, rexpr) \
  ASSIGN_OR_RETURN_IMPL(             \
      STATUS_MACROS_CONCAT_NAME(_status_or_value, __COUNTER__), lhs, rexpr);

}  // namespace absl

#endif  // OR_TOOLS_BASE_STATUS_MACROS_H_
