package main

import (
        "context"
        "log"
        "os"
        "time"

        "github.com/containerd/containerd"
        "github.com/containerd/containerd/cio"
        "github.com/containerd/containerd/namespaces"
        "github.com/containerd/containerd/oci"
)

func main() {
        // connect to containerd daemon socket
        client, err := containerd.New("/run/containerd/containerd.sock")
        if err != nil {
                log.Fatalf("failed to connect: %v", err)
        }
        defer client.Close()

        // use a namespace for isolation (matches kubelet behavior)
        ctx := namespaces.WithNamespace(context.Background(), "edge-ns")
        ctx, cancel := context.WithTimeout(ctx, 5*time.Minute)
        defer cancel()

        // pull and unpack image (uses default remote registry auth)
        image, err := client.Pull(ctx, "docker.io/library/nginx:stable",
                containerd.WithPullUnpack)
        if err != nil {
                log.Fatalf("pull failed: %v", err)
        }

        // create a container with a new snapshot from the image
        container, err := client.NewContainer(
                ctx,
                "nginx-edge",
                containerd.WithImage(image),
                containerd.WithNewSnapshot("nginx-snap-"+time.Now().Format("150405"), image),
                containerd.WithNewSpec(oci.WithImageConfig(image)),
        )
        if err != nil {
                log.Fatalf("create container failed: %v", err)
        }
        defer func() { _ = container.Delete(ctx, containerd.WithSnapshotCleanup) }()

        // create and start a task; attach stdio for logs
        task, err := container.NewTask(ctx, cio.NewCreator(cio.WithStdio))
        if err != nil {
                log.Fatalf("new task failed: %v", err)
        }
        defer func() { _, _ = task.Delete(ctx) }()

        if err := task.Start(ctx); err != nil {
                log.Fatalf("task start failed: %v", err)
        }

        // wait for a short duration to validate startup, then cleanup
        time.Sleep(3 * time.Second)

        statusC, err := task.Wait(ctx)
        if err != nil {
                log.Fatalf("wait failed: %v", err)
        }

        // send SIGTERM and cleanup gracefully
        if err := task.Kill(ctx, containerd.SIGTERM); err != nil {
                log.Printf("kill failed: %v", err)
        }
        <-statusC // ensure wait channel drained
}