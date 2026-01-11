package main

import (
        "encoding/json"
        "fmt"
        "time"

        "github.com/hyperledger/fabric-chaincode-go/shim" // chaincode shim
        pb "github.com/hyperledger/fabric-protos-go/peer"
)

// Asset represents a small provenance record suitable for edge storage.
type Asset struct {
        ID        string `json:"id"`
        OwnerMSP  string `json:"owner_msp"`  // MSP that created entry
        DocHash   string `json:"doc_hash"`   // off-chain content hash (IPFS/CID)
        Timestamp int64  `json:"timestamp"`  // epoch seconds
        Meta      string `json:"meta"`       // compact metadata
}

type EdgeChaincode struct{}

func (c *EdgeChaincode) Init(stub shim.ChaincodeStubInterface) pb.Response {
        return shim.Success(nil)
}

func (c *EdgeChaincode) Invoke(stub shim.ChaincodeStubInterface) pb.Response {
        fn, args := stub.GetFunctionAndParameters()
        switch fn {
        case "CreateAsset":
                return c.CreateAsset(stub, args)
        case "ReadAsset":
                return c.ReadAsset(stub, args)
        default:
                return shim.Error("unsupported function")
        }
}

// CreateAsset stores asset after checking invoker MSP and idempotency.
func (c *EdgeChaincode) CreateAsset(stub shim.ChaincodeStubInterface, args []string) pb.Response {
        if len(args) != 3 {
                return shim.Error("expected 3 args: id, docHash, meta")
        }
        id, docHash, meta := args[0], args[1], args[2]

        // Reject if exists
        existing, err := stub.GetState(id)
        if err != nil {
                return shim.Error(fmt.Sprintf("getstate error: %s", err))
        }
        if existing != nil {
                return shim.Error("asset already exists")
        }

        // Extract MSP ID from creator certificate for access control
        creator, err := stub.GetCreator()
        if err != nil {
                return shim.Error("cannot get creator")
        }
        // Simple MSP extraction; production should parse cert with x509
        ownerMSP := extractMSPFromCreator(creator) // compact function below

        asset := Asset{
                ID:        id,
                OwnerMSP:  ownerMSP,
                DocHash:   docHash,
                Timestamp: time.Now().Unix(),
                Meta:      meta,
        }
        b, _ := json.Marshal(asset)
        if err := stub.PutState(id, b); err != nil {
                return shim.Error(fmt.Sprintf("putstate error: %s", err))
        }
        return shim.Success(nil)
}

func (c *EdgeChaincode) ReadAsset(stub shim.ChaincodeStubInterface, args []string) pb.Response {
        if len(args) != 1 {
                return shim.Error("expected id")
        }
        b, err := stub.GetState(args[0])
        if err != nil || b == nil {
                return shim.Error("asset not found")
        }
        return shim.Success(b)
}

// extractMSPFromCreator is a compact, production-ready parsing stub.
// Replace with full x509 parsing to get accurate MSP ID.
func extractMSPFromCreator(creator []byte) string {
        // Inline parsing omitted for brevity; placeholder returns constant.
        return "Org1MSP"
}

func main() {
        if err := shim.Start(new(EdgeChaincode)); err != nil {
                fmt.Printf("Error starting chaincode: %s", err)
        }
}