#include "aos/network/timestamp_filter.h"

#include <chrono>
#include <iomanip>
#include <tuple>

#include "absl/log/vlog_is_on.h"
#include "absl/numeric/int128.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"

#include "aos/network/noncausal_timestamp_filter.h"
#include "aos/time/time.h"

namespace aos::message_bridge {
namespace {
namespace chrono = std::chrono;

void ClippedAverageFilterPrintHeader(FILE *fp) {
  fprintf(fp,
          "# time_since_start, sample_ns, filtered_offset, offset, "
          "velocity, filtered_velocity, velocity_contribution, "
          "sample_contribution, time_contribution\n");
}

}  // namespace

void TimestampFilter::Set(aos::monotonic_clock::time_point monotonic_now,
                          chrono::nanoseconds sample_ns) {
  const double sample =
      chrono::duration_cast<chrono::duration<double>>(sample_ns - base_offset_)
          .count();
  offset_ = sample;
  last_time_ = monotonic_now;
}

void TimestampFilter::Sample(aos::monotonic_clock::time_point monotonic_now,
                             chrono::nanoseconds sample_ns) {
  VLOG(2) << "  " << this << " Sample at " << monotonic_now << " is "
          << sample_ns.count() << "ns, Base is " << base_offset_.count();
  CHECK_GE(monotonic_now, last_time_)
      << ": " << this << " Being asked to filter backwards in time!";
  // Compute the sample offset as a double (seconds), taking into account the
  // base offset.
  const double sample =
      chrono::duration_cast<chrono::duration<double>>(sample_ns - base_offset_)
          .count();

  VelocitySample(monotonic_now, sample_ns);

  // This is our first sample.  Just use it.
  if (last_time_ == aos::monotonic_clock::min_time) {
    VLOG(1) << "  " << this << " First, setting offset to sample.";
    offset_ = sample;
    velocity_contribution_ = 0.0;
    sample_contribution_ = 0.0;
    time_contribution_ = 0.0;
  } else {
    const double dt = chrono::duration_cast<chrono::duration<double>>(
                          monotonic_now - last_time_)
                          .count();
    const double velocity_contribution = dt * filtered_velocity();
    velocity_contribution_ = velocity_contribution;
    offset_ += velocity_contribution;
    // Took less time to transmit, so clamp to it.
    if (sample < offset_) {
      offset_ = sample;
      velocity_contribution_ = 0.0;
      sample_contribution_ = 0.0;
      time_contribution_ = 0.0;
    } else {
      // We split things up into 2 portions.
      //  1) Each sample has information.  Correct some using it.
      //  2) We want to keep a decent time constant if the sample rate slows.
      //     Take time since the last sample into account.
      //
      // There are other challenges.  If every long sample is followed by a
      // short sample, we will over-weight the long samples.
      //
      // In the end, this algorithm does ok for reasonably well conditioned
      // data, but occasionally violates causality when the network delay goes
      // from large to small suddenly.  Good enough for a real time estimate,
      // but not great for where we need 100% accuracy replaying logs.

      // Time constant for the low pass filter in seconds.
      constexpr double kTau = 1.0;

      constexpr double kClampPositive = 0.0005;
      // Scale the slew rate clamp by dt.  This helps when there is an outage.
      const double clamp_positive = kClampPositive * dt;
      clamp_ = clamp_positive;

      {
        // 1)
        constexpr double kAlpha = 0.0005;
        // This formulation is more numerically precise.
        const double sample_contribution =
            std::min(kAlpha * (sample - offset_), 0.000001);
        offset_ = offset_ + sample_contribution;
        sample_contribution_ = sample_contribution;
      }

      {
        // 2)
        //
        // 1-e^(-t/tau) -> alpha
        const double alpha = -std::expm1(-dt / kTau);

        // Clamp to clamp_positive ms to reduce the effect of wildly large
        // samples.
        const double time_contribution =
            std::min(alpha * (sample - offset_), clamp_positive);
        offset_ = offset_ + time_contribution;

        time_contribution_ = time_contribution;
      }

      VLOG(2) << "  " << this << " filter sample is " << offset_;
    }
  }

  last_time_ = monotonic_now;
}

void TimestampFilter::set_base_offset(chrono::nanoseconds base_offset) {
  offset_ -= chrono::duration_cast<chrono::duration<double>>(base_offset -
                                                             base_offset_)
                 .count();
  base_offset_ = base_offset;
  // Clear everything out to avoid any numerical precision problems.
  last_time_ = aos::monotonic_clock::min_time;
  last_velocity_sample_time_ = aos::monotonic_clock::min_time;
  velocity_ = 0;
  filtered_velocity_ = 0;
}

void TimestampFilter::Reset() {
  offset_ = 0;

  last_time_ = aos::monotonic_clock::min_time;
  base_offset_ = chrono::nanoseconds(0);

  last_velocity_sample_time_ = aos::monotonic_clock::min_time;
}

void TimestampFilter::VelocitySample(
    aos::monotonic_clock::time_point monotonic_now,
    chrono::nanoseconds sample_ns) {
  if (last_velocity_sample_time_ == aos::monotonic_clock::min_time) {
    last_velocity_sample_time_ = monotonic_now;
    last_velocity_sample_ns_ = sample_ns;
    velocity_ = 0.0;
    state_velocity_ = 0.0;
    filtered_velocity_ = 0.0;
    last_sample_ns_ = sample_ns;
    filtered_velocity_time_ = 0.5;
  } else {
    chrono::duration<double> elapsed_time =
        chrono::duration_cast<chrono::duration<double>>(
            monotonic_now - last_velocity_sample_time_);

    velocity_ = chrono::duration_cast<chrono::duration<double>>(sample_ns -
                                                                last_sample_ns_)
                    .count() /
                chrono::duration_cast<chrono::duration<double>>(monotonic_now -
                                                                last_time_)
                    .count();
    if (sample_ns - last_velocity_sample_ns_ <
        chrono::duration_cast<chrono::nanoseconds>(
            chrono::duration<double>(elapsed_time.count() * kMaxVelocity()))) {
      state_velocity_ = chrono::duration_cast<chrono::duration<double>>(
                            sample_ns - last_velocity_sample_ns_)
                            .count() /
                        elapsed_time.count();
      last_velocity_sample_ns_ = sample_ns;
      last_velocity_sample_time_ = monotonic_now;

      constexpr double kSampleTime = 1.0;

      // Limit the weight of historical time.  This makes it so we slow down
      // over time, but don't slow down forever.
      const double clamped_time =
          std::min(kSampleTime, filtered_velocity_time_);

      // Compute a weighted average of the previous velocity and the new
      // velocity.  The filtered velocity is weighted by a time it has been
      // accumulated over, and the sample velocity is purely based on the
      // elapsed time.
      const double unclamped_velocity =
          (filtered_velocity_ * clamped_time +
           std::clamp(state_velocity_, -0.1, kMaxVelocity()) *
               elapsed_time.count()) /
          (clamped_time + elapsed_time.count());

      filtered_velocity_ =
          std::clamp(unclamped_velocity, -kMaxVelocity(), kMaxVelocity());
      filtered_velocity_time_ += elapsed_time.count();
    }
  }
  last_sample_ns_ = sample_ns;
}

void ClippedAverageFilter::SetFwdCsvFileName(std::string_view name) {
  fwd_csv_file_name_ = name;
  fwd_fp_ = fopen(absl::StrCat(fwd_csv_file_name_, ".csv").c_str(), "w");
  ClippedAverageFilterPrintHeader(fwd_fp_);
}

void ClippedAverageFilter::SetRevCsvFileName(std::string_view name) {
  rev_csv_file_name_ = name;
  rev_fp_ = fopen(absl::StrCat(rev_csv_file_name_, ".csv").c_str(), "w");
  ClippedAverageFilterPrintHeader(rev_fp_);
}

void ClippedAverageFilter::set_first_fwd_time(
    aos::monotonic_clock::time_point time) {
  first_fwd_time_ = time;
  if (fwd_fp_) {
    fwd_fp_ = freopen(NULL, "wb", fwd_fp_);
    ClippedAverageFilterPrintHeader(fwd_fp_);
  }
}

void ClippedAverageFilter::set_first_rev_time(
    aos::monotonic_clock::time_point time) {
  first_rev_time_ = time;
  if (rev_fp_) {
    rev_fp_ = freopen(NULL, "wb", rev_fp_);
    ClippedAverageFilterPrintHeader(rev_fp_);
  }
}

void ClippedAverageFilter::FwdSet(
    aos::monotonic_clock::time_point monotonic_now,
    chrono::nanoseconds sample_ns) {
  VLOG(2) << "Fwd Set";
  fwd_.Set(monotonic_now, sample_ns);
  Update(monotonic_now, &last_fwd_time_);
}

void ClippedAverageFilter::FwdSample(
    aos::monotonic_clock::time_point monotonic_now,
    chrono::nanoseconds sample_ns) {
  VLOG(1) << &fwd_ << " Fwd sample now " << monotonic_now << " sample "
          << sample_ns.count();
  fwd_.Sample(monotonic_now, sample_ns);
  Update(monotonic_now, &last_fwd_time_);

  if (fwd_fp_ != nullptr) {
    if (first_fwd_time_ == aos::monotonic_clock::min_time) {
      first_fwd_time_ = monotonic_now;
    }
    fprintf(
        fwd_fp_,
        "%.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %9.9f, %9.9f, "
        "%9.9f, %.9f\n",
        chrono::duration_cast<chrono::duration<double>>(monotonic_now -
                                                        first_fwd_time_)
            .count(),
        chrono::duration_cast<chrono::duration<double>>(sample_ns).count(),
        fwd_.offset() + fwd_.base_offset_double(),
        chrono::duration_cast<chrono::duration<double>>(offset()).count(),
        fwd_.velocity(), fwd_.filtered_velocity(),
        chrono::duration_cast<chrono::duration<double>>(
            monotonic_now.time_since_epoch())
            .count(),
        offset_velocity_, fwd_.last_velocity_sample(),
        fwd_.velocity_contribution(), fwd_.sample_contribution(),
        fwd_.time_contribution(), fwd_.clamp());
    fflush(fwd_fp_);
  }
}

void ClippedAverageFilter::RevSet(
    aos::monotonic_clock::time_point monotonic_now,
    chrono::nanoseconds sample_ns) {
  VLOG(2) << "Rev set";
  rev_.Set(monotonic_now, sample_ns);
  Update(monotonic_now, &last_rev_time_);
}

void ClippedAverageFilter::RevSample(
    aos::monotonic_clock::time_point monotonic_now,
    chrono::nanoseconds sample_ns) {
  VLOG(1) << "Rev sample";
  rev_.Sample(monotonic_now, sample_ns);
  Update(monotonic_now, &last_rev_time_);

  if (rev_fp_ != nullptr) {
    if (first_rev_time_ == aos::monotonic_clock::min_time) {
      first_rev_time_ = monotonic_now;
    }
    fprintf(
        rev_fp_,
        "%.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %9.9f, %9.9f, "
        "%9.9f, %.9f\n",
        chrono::duration_cast<chrono::duration<double>>(monotonic_now -
                                                        first_rev_time_)
            .count(),
        chrono::duration_cast<chrono::duration<double>>(sample_ns).count(),
        rev_.offset() + rev_.base_offset_double(),
        chrono::duration_cast<chrono::duration<double>>(offset()).count(),
        rev_.velocity(), rev_.filtered_velocity(),
        chrono::duration_cast<chrono::duration<double>>(
            monotonic_now.time_since_epoch())
            .count(),
        offset_velocity_, rev_.last_velocity_sample(),
        rev_.velocity_contribution(), rev_.sample_contribution(),
        rev_.time_contribution(), rev_.clamp());
    fflush(rev_fp_);
  }
}

void ClippedAverageFilter::set_base_offset(chrono::nanoseconds base_offset) {
  offset_ -= chrono::duration_cast<chrono::duration<double>>(base_offset -
                                                             base_offset_)
                 .count();
  fwd_.set_base_offset(base_offset);
  rev_.set_base_offset(-base_offset);
  base_offset_ = base_offset;
  last_fwd_time_ = aos::monotonic_clock::min_time;
  last_rev_time_ = aos::monotonic_clock::min_time;

  if (fwd_fp_) {
    fprintf(fwd_fp_, "# Closing and opening\n");
    fclose(fwd_fp_);
    fwd_fp_ = NULL;
    SetFwdCsvFileName(fwd_csv_file_name_);
  }
  if (rev_fp_) {
    fprintf(rev_fp_, "# Closing and opening\n");
    fclose(rev_fp_);
    rev_fp_ = NULL;
    SetRevCsvFileName(rev_csv_file_name_);
  }
}

void ClippedAverageFilter::Reset() {
  base_offset_ = chrono::nanoseconds(0);
  offset_ = 0;
  offset_velocity_ = 0;

  last_fwd_time_ = aos::monotonic_clock::min_time;
  last_rev_time_ = aos::monotonic_clock::min_time;
  first_fwd_time_ = aos::monotonic_clock::min_time;
  first_rev_time_ = aos::monotonic_clock::min_time;

  fwd_.Reset();
  rev_.Reset();

  if (fwd_fp_) {
    fclose(fwd_fp_);
    fwd_fp_ = NULL;
    SetFwdCsvFileName(fwd_csv_file_name_);
  }
  if (rev_fp_) {
    fclose(rev_fp_);
    rev_fp_ = NULL;
    SetRevCsvFileName(rev_csv_file_name_);
  }
}

void ClippedAverageFilter::Update(
    aos::monotonic_clock::time_point monotonic_now,
    aos::monotonic_clock::time_point *last_time) {
  // ta = t + offseta
  // tb = t + offsetb
  // fwd sample => ta - tb + network -> offseta - offsetb + network
  // rev sample => tb - ta + network -> offsetb - offseta + network
  const double hard_max = fwd_.offset();
  const double hard_min = -rev_.offset();
  const double average = (hard_max + hard_min) / 2.0;
  VLOG(2) << this << "  Max(fwd) " << hard_max << " min(rev) " << hard_min;
  // We don't want to clip the offset to the hard min/max.  We really want to
  // keep it within a band around the middle.  ratio of 0.3 means stay within
  // +- 0.15 of the middle of the hard min and max.
  constexpr double kBand = 0.3;
  const double max = average + kBand / 2.0 * (hard_max - hard_min);
  const double min = average - kBand / 2.0 * (hard_max - hard_min);

  // Update regardless for the first sample from both the min and max.
  if (*last_time == aos::monotonic_clock::min_time) {
    VLOG(1) << this << "  No last time " << average;
    offset_ = average;
    offset_velocity_ = 0.0;
  } else {
    // Do just a time constant based update.  We can afford to be slow here
    // for smoothness.
    constexpr double kTau = 5.0;
    constexpr double kTauVelocity = 0.75;
    const double dt = chrono::duration_cast<chrono::duration<double>>(
                          monotonic_now - *last_time)
                          .count();
    const double alpha = -std::expm1(-dt / kTau);
    const double velocity_alpha = -std::expm1(-dt / kTauVelocity);

    // Clamp it such that it remains in the min/max bounds.
    offset_ += std::clamp(offset_velocity_, -kMaxVelocity(), kMaxVelocity()) *
               dt / 2.0;
    offset_ = std::clamp(offset_ - alpha * (offset_ - average), min, max);

    offset_velocity_ =
        offset_velocity_ -
        velocity_alpha *
            (offset_velocity_ -
             (fwd_.filtered_velocity() - rev_.filtered_velocity()) / 2.0);

    VLOG(2) << this << "  last time " << offset_;
  }
  *last_time = monotonic_now;

  if (sample_pointer_ != nullptr) {
    // TODO(austin): Probably shouldn't do the update if we don't have fwd and
    // reverse samples.
    if (!MissingSamples()) {
      *sample_pointer_ = offset_;
      VLOG(1) << this << " Updating sample to " << offset_;
    } else {
      VLOG(1) << this << " Don't have both samples.";
      if (last_fwd_time_ == aos::monotonic_clock::min_time) {
        VLOG(1) << this << " Missing forward";
      }
      if (last_rev_time_ == aos::monotonic_clock::min_time) {
        VLOG(1) << this << " Missing reverse";
      }
    }
  }
}

}  // namespace aos::message_bridge
