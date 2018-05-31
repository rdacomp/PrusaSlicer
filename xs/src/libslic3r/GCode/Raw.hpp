#ifndef slic3r_GCode_Raw_hpp_
#define slic3r_GCode_Raw_hpp_

#include "../libslic3r.h"

#include <algorithm>
#include <assert.h>
#include <math.h>
#include <vector>

#if 0
    #define DEBUG
    #define _DEBUG
    #undef NDEBUG
#endif
#include <assert.h>

namespace Slic3r {
namespace GCode {
namespace Raw {

struct GCodeVec {
	GCodeVec() {}
	GCodeVec(double x, double y, double z, double e) { data[0] = x; data[1] = y; data[2] = z; data[3] = e; }
	GCodeVec(const double pos[4]) { data[0] = pos[0]; data[1] = pos[1]; data[2] = pos[2]; data[3] = pos[3]; }
	GCodeVec(const float  pos[4]) { data[0] = pos[0]; data[1] = pos[1]; data[2] = pos[2]; data[3] = pos[3]; }

	double data[4];
	double& operator[](size_t idx) { return data[idx]; }
	const double& operator[](size_t idx) const { return data[idx]; }

	GCodeVec to_grid(const GCodeVec &resolution) const {
		GCodeVec out;
		for (size_t i = 0; i < 4; ++ i)
			out[i] = floor(this->data[i] / resolution[i] + 0.5) * resolution[i];
	}
};

inline GCodeVec operator-(const GCodeVec &v1, const GCodeVec &v2)
{ 
	return GCodeVec(v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2], v2[3] - v1[3]);
}

struct RawGCodeEvent {
	enum {
		X_STEP = 0x01,
		Y_STEP = 0x02,
		Z_STEP = 0x04,
		E_STEP = 0x08,
		X_DIR  = 0x10,
		Y_DIR  = 0x20,
		Z_DIR  = 0x40,
		E_DIR  = 0x80,
		X_POS  = X_STEP,
		X_NEG  = X_STEP | X_DIR,
		Y_POS  = Y_STEP,
		Y_NEG  = Y_STEP | Y_DIR,
		Z_POS  = Z_STEP,
		Z_NEG  = Z_STEP | Z_DIR,
		E_POS  = E_STEP,
		E_NEG  = E_STEP | E_DIR,
	};
	unsigned char bits;
	double        time;
};

inline void reduce_events(std::vector<RawGCodeEvent> &events, double time_epsilon)
{
	// Sort by the event time.
	std::sort(events.begin(), events.end(), [](const RawGCodeEvent &e1, const RawGCodeEvent &e2) { return e1.time < e2.time; });
	// Group events with too close times.
	int n = int(events.size());
	int k = 0;
	for (int i = 0; i < n;) {
		int j = i + 1;
		double time_max = events[i].time + time_epsilon;
		for (; j < n && events[j].time <= time_max; ++ j);
		unsigned char bits = 0;
		for (int l = i; l < j; ++ l)
			bits |= events[l].bits;
		events[k].bits = bits;
		events[k].time = (j > i + 1) ? (0.5 * (events[i].time + events[j - 1].time)) : events[i].time;
		++ k;
		i = j;
	}
	if (k < n)
		events.erase(events.begin() + k, events.end());
}


struct RawGCodeBlock {
	RawGCodeBlock() { this->reset(); }

	void reset() { data[0] = 128; data[1] = 0; }

	void add_event(unsigned char bits, unsigned char timer_delta) {
		assert(this->num_events() < max_num_events);
		int pos = this->num_events() * 2 + 2;
		this->data[pos] = bits;
		this->data[pos + 1] = timer_delta;
		this->data[1] += 2;
	}

	size_t num_events() const { return (size_t)(this->data[1] >> 1); }
	bool   full()	    const { return this->num_events() == max_num_events; }
	bool   empty()      const { return this->num_events() == 0; }

	size_t fwrite(FILE *fout) {
		// Finish the line.
		int len = 2 + this->num_events() * 2;
		this->data[len ++] = '\n';
		this->data[len] = 0;
		return ::fwrite(this->data, len, 1, fout);
	}

	static const int max_num_events = 40;
	unsigned char data[max_num_events * 2 + 4];
};

struct TrapezoidalProfileParameters {
	GCodeVec	pos_entry;
	GCodeVec	pos_exit;
	double		feedrate_entry;
	double		feedrate_target;
	double		feedrate_exit;
	double		acceleration;

	// For S-curve profiles:
	double		average_acceleration = 0.;

	GCodeVec	pos_delta() const { return pos_exit - pos_entry; }
};

struct MachineLimits {
	double		max_feedrate_xy;
	double		max_feedrate_z;
	double		max_feedrate_e;
	double		max_acceleration_xy;
	double		max_acceleration_z;
	double		max_acceleration_e;
	bool		s_curves = false;
	double		jerk_xy;
	double		jerk_z;
	double		jerk_e;
	GCodeVec    step_resolution;
	double      timer_frequency_Hz;
};

struct PrintParameters
{
	double layer_height;
	double first_layer_height;
	double extrusion_width;
};

template<typename FUN>
inline double secant(FUN f, double x1, double x2, double eps)
{
	double fx1 = f(x1);
	double fx2 = f(x2);
	assert(fx1 * fx2 <= 0);
	for (;;) {
		double x0  = (x1 * fx2 - x2 * fx1) / (fx2 - fx1);
		double fx0 = f(x0);
		if (std::abs(fx0) < eps)
			return x0;
		x1  = x2;
		fx1 = fx2;
		x2  = x0;
		fx2 = fx0;
	}
}

template<typename FUN_DISTANCE, typename FUN_VELOCITY>
inline double newton_raphson(FUN_DISTANCE dist, FUN_VELOCITY vel, const double t0, double d0, const double t1, double d1, double d)
{
	const double eps = 0.00001;


	//FIXME
	return secant([dist, d](double t){ return dist(t) - d; }, t0, t1, eps);


	double t = lerp(t0, t1, (d - d0) / (d1 - d0));
	for (;;) {
		double tt = d - dist(t) / vel(t);
		// double err = std::abs(t - tt);
		double err = d - dist(tt);
		t = tt;
		if (err < 0.00001)
			break;
	}
	if (t < t0 - eps || t > t1 + eps) {
		// The root search diverged.
		// Try the regula falsi method;
		t = secant([dist, d](double t){ return dist(t) - d; }, t0, t1, eps);
		return t;
	}
	if (t < t0)
		return t0;
	if (t > t1)
		return t1;
	return t;
}

struct TrapezoidalProfile : public TrapezoidalProfileParameters
{
	// Distance of the move (Euclidian distance if moving, filament distance if just retracting / deretracting).
	double  distance;
	// Total time spent on this segment.
	double  time;
	// Length of the acceleration phase.
	double  accelerate_until_distance;
	double  accelerate_until_time;
	// Position at which to start decelerating.
	double  decelerate_after_distance;
	double  decelerate_after_time;
	// Feedrate reached.
	double  feedrate_cruise;

	// For the S-curve planning:
	double	t1;
	double	d1;
	double	t2;
	double	d2;
	double  t5;
	double  d5;
	double  t6;
	double  d6;
	// Average jerk
	double  Ja_enter;
	double	Ja_exit;

	void	initialize()
	{
		double acc = this->has_s_curve() ? this->average_acceleration : this->acceleration;

		this->distance = this->move_length();
		double accelerate_distance = acceleration_distance(this->feedrate_entry, this->feedrate_target, acc);
		double decelerate_distance = acceleration_distance(this->feedrate_target, this->feedrate_exit, -acc);
		double cruise_distance = distance - accelerate_distance - decelerate_distance;
		// Not enough space to reach the nominal feedrate.
		// This means no cruising, and we'll have to use intersection_distance() to calculate when to abort acceleration 
		// and start braking in order to reach the exit_feedrate exactly at the end of this block.
		if (cruise_distance < 0.0f) {
			//FIXME before this is adjusted for the S-curves:
			if (this->has_s_curve()) {
				acc = this->acceleration = this->average_acceleration;
				this->average_acceleration = 0;
			}

			accelerate_distance = clamp(0.0, distance, intersection_distance(this->feedrate_entry, this->feedrate_exit, acc, this->distance));
			decelerate_distance = this->distance - accelerate_distance;
			cruise_distance = 0.0f;
			this->feedrate_cruise = speed_from_distance(this->feedrate_entry, accelerate_distance, acc);
		} else
			this->feedrate_cruise = this->feedrate_target;
		this->accelerate_until_distance = accelerate_distance;
		this->decelerate_after_distance = accelerate_distance + cruise_distance;
		this->accelerate_until_time     = acceleration_time(this->feedrate_entry, acc, accelerate_distance);
		this->decelerate_after_time		= cruise_distance / this->feedrate_cruise + this->accelerate_until_time;
		this->time                      = this->decelerate_after_time + acceleration_time(this->feedrate_exit, acc, decelerate_distance);


		if (this->has_s_curve()) {
			// Fill in the S-curve profile.
			// see https://www.parkermotion.com/manuals/6k/6K_PG_RevB_Chp5_CustomProfiling.pdf
			// page 4
			double V0 = this->feedrate_entry;
			double V  = this->feedrate_cruise - V0;
			double A  = this->acceleration;
			double AA = this->average_acceleration;
			double Ja = sqr(A) * AA / (V * (A - AA));
			double T1 = A / Ja;
			double D1 = V0 * T1 + Ja * sqr(T1) * T1 / 6.;
			double T2 = V / AA - A / Ja;
			double V1 = V0 + 0.5 * Ja * sqr(T1);
			double D2 = D1 + 0.5 * A * (T2 - T1) + V1 * (T2 - T1);
			this->Ja_enter = Ja;
			this->t1 = T1;
			this->d1 = D1;
			this->t2 = T2;
			this->d2 = D2;

			assert(t1 > 0);
			assert(t1 < t2 + 1e-7);
			assert(t2 < this->accelerate_until_time);
			assert(d1 > 0);
			assert(d1 < d2 + 1e-7);
			assert(d2 < this->accelerate_until_distance);

			V0 = this->feedrate_exit;
			V = this->feedrate_cruise - V0;
			Ja = sqr(A) * AA / (V * (A - AA));
			T1 = A / Ja;
			D1 = V0 * T1 + Ja * sqr(T1) * T1 / 6.;
			T2 = V / AA - A / Ja;
			V1 = V0 + 0.5 * Ja * sqr(T1);
			D2 = D1 + 0.5 * A * (T2 - T1) + V1 * (T2 - T1);
			this->Ja_exit = Ja;
			this->t6 = this->time - T1;
			this->d6 = this->distance - D1;
			this->t5 = this->time - T2;
			this->d5 = this->distance - D2;

			assert(t5 > this->decelerate_after_time);
			assert(t5 < t6 + 1e-7);
			assert(t6 < this->time);
			assert(d5 > this->decelerate_after_distance);
			assert(d5 < d6 + 1e-7);
			assert(d6 < this->distance);
		}
	}

	std::vector<RawGCodeEvent> events(const GCodeVec &step_resolution, const double timer_freq_Hz) const {
		std::vector<RawGCodeEvent> out;
		
		// Integer steps from the start and end position.
		int ipos_entry[4];
		int ipos_exit[4];
		int isteps[4];
		for (size_t i = 0; i < 4; ++ i) {
			ipos_entry[i] = int(floor(this->pos_entry[i] / step_resolution[i] + 0.5));
			ipos_exit [i] = int(floor(this->pos_exit [i] / step_resolution[i] + 0.5));
			isteps    [i] = ipos_exit[i] - ipos_entry[i];
		}
		assert(isteps[0] != 0 || isteps[1] != 0 || isteps[2] != 0 || isteps[3] != 0);

		// Discretize each axis separately.
		for (size_t iaxis = 0; iaxis < 4; ++ iaxis) {
			// Set the axis direction;
			int cnt = std::abs(isteps[iaxis]);
			if (cnt == 0)
				continue;
			// Step size, in Euclidian coordinates.
			double step_size = this->distance / double(cnt);
			RawGCodeEvent evt;
			// Set the axis code.
			evt.bits = 1 << iaxis;
			if (cnt != isteps[iaxis])
				// Movement in the negative direction.
				evt.bits |= (0x10 << iaxis);
			for (int istep = 0; istep < cnt; ++ istep) {
				double d = (double)(istep + 1) * step_size;
				double t;
				if (d < accelerate_until_distance) {
					// Acceleration phase.
					if (this->has_s_curve()) {
						double V0 = this->feedrate_entry;
						double V  = this->feedrate_cruise - this->feedrate_entry;
						double A  = this->acceleration;
						double AA = this->average_acceleration;
						double Ja = this->Ja_enter;
						if (d < this->d1) {
							t = newton_raphson(
								// Distance from time
								[V0, Ja](double t){ return V0 * t + Ja * sqr(t) * t / 6.; },
								// Velocity from time
								[V0, Ja](double t){ return V0 + 0.5 * Ja * sqr(t); },
								// Time span, distance span, linear interpolation is used for the initial estimate.
								0., 0., this->t1, this->d1, d);
							assert(istep == 0 || out.back().time < t);
						} else if (d < this->d2) {
							double T1 = this->t1;
							double V1 = V0 + 0.5 * Ja * sqr(T1);
							double D1 = this->d1;
							t = newton_raphson(
								// Distance from time
								[A, V1, D1, T1](double t){ double dt = t - T1; return D1 + 0.5 * A * sqr(dt) + V1 * dt; },
								// Velocity from time
								[A, V1, T1](double t){ return A * (t - T1) + V1; },
								// Time span, distance span, linear interpolation is used for the initial estimate.
								T1, D1, this->t2, this->d2, d);
							assert(istep == 0 || out.back().time < t);
						} else {
							double T3 = this->accelerate_until_time;
							double D3 = this->accelerate_until_distance;
							double V = this->feedrate_cruise;
							t = newton_raphson(
								// Distance from time
								[Ja, A, D3, V, T3](double t){ double dt = T3 - t; return D3 + Ja * sqr(dt) * dt / 6.0 - V * dt; },
								// Velocity from time
								[Ja, A, V, T3](double t){ return - 0.5 * Ja * sqr(T3 - t) + V; },
								// Time span, distance span, linear interpolation is used for the initial estimate.
								this->t2, this->d2, T3, D3, d);
							assert(istep == 1 || out.back().time < t);
						}
					} else {
						t = acceleration_time(this->feedrate_entry, this->acceleration, d);
						assert(istep == 0 || out.back().time < t);
					}
				} else if (d > decelerate_after_distance) {
					// Deceleration phase.
					if (this->has_s_curve()) {
						double V0 = this->feedrate_exit;
						double V  = this->feedrate_cruise - this->feedrate_exit;
						double A  = this->acceleration;
						double AA = this->average_acceleration;
						double Ja = this->Ja_exit;
						if (d > this->d6) {
							t = this->time - newton_raphson(
								// Distance from time
								[V0, Ja](double t){ return V0 * t + Ja * sqr(t) * t / 6.; },
								// Velocity from time
								[V0, Ja](double t){ return V0 + 0.5 * Ja * sqr(t); },
								// Time span, distance span, linear interpolation is used for the initial estimate.
								0., 0., this->time - this->t6, this->distance - this->d6, this->distance - d);
							assert(istep == 0 || out.back().time < t);
						} else if (d > this->d5) {
							double T1 = this->time - this->t6;
							double V1 = V0 + 0.5 * Ja * sqr(T1);
							double D1 = this->distance - this->d6;
							t = this->time - newton_raphson(
								// Distance from time
								[A, V1, D1, T1](double t){ double dt = t - T1; return D1 + 0.5 * A * sqr(dt) + V1 * dt; },
								// Velocity from time
								[A, V1, T1](double t){ return A * (t - T1) + V1; },
								// Time span, distance span, linear interpolation is used for the initial estimate.
								T1, D1, this->time - this->t5, this->distance - this->d5, this->distance - d);
							assert(istep == 0 || out.back().time < t);
						} else {
							double T3 = this->time - this->decelerate_after_time;
							double D3 = this->distance - this->decelerate_after_distance;
							double V = this->feedrate_cruise;
							t = this->time - newton_raphson(
								// Distance from time
								[Ja, A, D3, V, T3](double t){ double dt = T3 - t; return D3 + Ja * sqr(dt) * dt / 6.0 - V * dt; },
								// Velocity from time
								[Ja, A, V, T3](double t){ return - 0.5 * Ja * sqr(T3 - t) + V; },
								// Time span, distance span, linear interpolation is used for the initial estimate.
								this->time - this->t5, this->distance - this->d5, T3, D3, this->distance - d);
							assert(istep == 0 || out.back().time < t);
						}
					} else {
						t = this->time - acceleration_time(this->feedrate_exit, this->acceleration, this->distance - d);
						assert(istep == 0 || out.back().time < t);
					}
				}
				else {
					// Steady phase (aka cruising).
					t = this->accelerate_until_time + (d - this->accelerate_until_distance) / this->feedrate_cruise;
					assert(istep == 0 || out.back().time < t);
				}
				assert(istep == 0 || out.back().time < t);
				evt.time = t;
				out.emplace_back(evt);
			}
		}

		// Sort the events.
		std::sort(out.begin(), out.end(), [](const RawGCodeEvent &e1, const RawGCodeEvent &e2) { return e1.time < e2.time; });
		return out;
	}

	bool has_s_curve() const { return this->average_acceleration >= 0.5 * this->acceleration && this->average_acceleration < 0.99 * this->acceleration; }

private:
	double move_length() const {
		GCodeVec pos_delta = this->pos_delta();
		double length = ::sqrt(sqr(pos_delta[0]) + sqr(pos_delta[1]) + sqr(pos_delta[2]));
		return (length > 0.0) ? length : std::abs(pos_delta[3]);
	}
	static double acceleration_distance(double initial_rate, double target_rate, double acceleration)
		{ return (acceleration == 0.0) ? 0.0 : (sqr(target_rate) - sqr(initial_rate)) / (2.0 * acceleration); }
	static double acceleration_time(double initial_rate, double acceleration, double distance)
		{ return (acceleration == 0.0) ? distance / initial_rate : (sqrt(sqr(initial_rate) + 2. * acceleration * distance) - initial_rate) / acceleration; }
	static double intersection_distance(double initial_rate, double final_rate, double acceleration, double distance)
		{ return (acceleration == 0.0) ? 0.0 : (2.0 * acceleration * distance - sqr(initial_rate) + sqr(final_rate)) / (4.0 * acceleration); }
	static double speed_from_distance(double initial_feedrate, double distance, double acceleration) {
		// to avoid invalid negative numbers due to numerical imprecision 
		double value = std::max(0.0, sqr(initial_feedrate) + 2.0 * acceleration * distance);
		return ::sqrt(value);
	}
};

extern std::vector<RawGCodeEvent> discretize_trapezoidal_profiles(const std::vector<TrapezoidalProfile> &moves, MachineLimits &limits);
extern void fwrite_gcode_events(FILE *file, std::vector<RawGCodeEvent> &events, const double timer_frequency_Hz);
extern void fwrite_discretize_trapezoidal_profiles(FILE *file, const std::vector<TrapezoidalProfile> &moves, MachineLimits &limits);

} // namespace Raw
} // namespace GCode
} // namespace Slic3r

#endif /* slic3r_GCode_Raw_hpp_ */

