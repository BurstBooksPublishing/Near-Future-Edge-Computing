#include 
#include 

typedef int32_t q16_t;                // Q16.16 fixed-point
#define Q16_ONE (1<<16)
#define TO_Q16(x) ((q16_t)((x) * (double)Q16_ONE))
#define MUL_Q16(a,b) (q16_t)(((int64_t)(a)*(b))>>16)

// Neuron parameters (Q16.16)
typedef struct {
    q16_t alpha;      // leak factor per tick
    q16_t R_I;        // R * input gain
    q16_t V_th;       // threshold
    q16_t V_rest;     // resting potential
} lif_params_t;

typedef struct {
    q16_t V;          // membrane potential
    lif_params_t p;
    void (*on_spike)(uint32_t neuron_id); // user callback
    uint32_t id;
} lif_neuron_t;

// Initialize neuron
static inline void lif_init(lif_neuron_t *n, uint32_t id, lif_params_t p,
                            void (*cb)(uint32_t)) {
    n->id = id;
    n->p = p;
    n->V = p.V_rest;
    n->on_spike = cb;
}

// Call every fixed tick with input current in Q16.16
static inline void lif_step(lif_neuron_t *n, q16_t I_in) {
    // V = alpha*V + (1-alpha)*R*I - V_th*S
    q16_t term1 = MUL_Q16(n->p.alpha, n->V);
    q16_t one_minus_alpha = Q16_ONE - n->p.alpha;
    q16_t term2 = MUL_Q16(one_minus_alpha, MUL_Q16(n->p.R_I, I_in));
    n->V = term1 + term2;
    if (n->V >= n->p.V_th) {
        // emit spike and reset by subtracting threshold
        if (n->on_spike) n->on_spike(n->id);
        n->V -= n->p.V_th;
        if (n->V < n->p.V_rest) n->V = n->p.V_rest; // clamp
    }
}