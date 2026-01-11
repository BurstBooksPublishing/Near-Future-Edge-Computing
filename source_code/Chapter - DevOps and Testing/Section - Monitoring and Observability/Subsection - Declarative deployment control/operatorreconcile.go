package controllers

import (
        "context"
        "time"

        "k8s.io/apimachinery/pkg/types"
        ctrl "sigs.k8s.io/controller-runtime"
        "sigs.k8s.io/controller-runtime/pkg/client"
        "sigs.k8s.io/controller-runtime/pkg/controller/controllerutil"

        "github.com/prometheus/client_golang/prometheus"
)

var (
        reconciles = prometheus.NewCounter(prometheus.CounterOpts{
                Name: "operator_reconcile_total",
                Help: "Total reconcile loops executed.",
        })
        driftSeconds = prometheus.NewGauge(prometheus.GaugeOpts{
                Name: "operator_drift_seconds",
                Help: "Seconds between observed and desired state for resource.",
        })
)

func init() {
        prometheus.MustRegister(reconciles, driftSeconds)
}

type GatewayReconciler struct {
        client.Client
        Scheme *runtime.Scheme
}

func (r *GatewayReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
        start := time.Now()
        reconciles.Inc()

        var gw MyGatewayCRD
        if err := r.Get(ctx, req.NamespacedName, &gw); err != nil {
                return ctrl.Result{}, client.IgnoreNotFound(err) // resource deleted or not found
        }

        // compute desired Deployment from CRD
        desired := buildDeploymentForGateway(&gw)

        // set owner reference for garbage collection
        if err := controllerutil.SetControllerReference(&gw, desired, r.Scheme); err != nil {
                return ctrl.Result{RequeueAfter: time.Second * 5}, err
        }

        // server-side apply to avoid patch conflicts across controllers
        applyOptions := []client.PatchOption{client.ForceOwnership, client.FieldOwner("gateway-operator")}
        if err := r.Patch(ctx, desired, client.Apply, applyOptions...); err != nil {
                // transient error: requeue with exponential backoff
                return ctrl.Result{RequeueAfter: backoffFor(err, start)}, err
        }

        // measure drift: time between last-applied and observed generation
        drift := time.Since(start).Seconds()
        driftSeconds.Set(drift)

        // ready: requeue for periodic reconciliation to detect drift and external changes
        return ctrl.Result{RequeueAfter: time.Minute * 1}, nil
}

func backoffFor(err error, start time.Time) time.Duration {
        // simple exponential backoff bounded to 30s
        elapsed := time.Since(start)
        d := time.Second * time.Duration(1< 30*time.Second {
                return 30 * time.Second
        }
        return d
}