#!/usr/bin/env python3
"""Train a deterministic terrain segmentation model."""
import argparse, json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
from lunabot_ml.config import config_sha256, load_config, seed_everything
from lunabot_ml.dataset import create_dataset
from lunabot_ml.losses import build_loss
from lunabot_ml.models import build_model

def main():
    p=argparse.ArgumentParser(); p.add_argument("--config",type=Path,required=True); p.add_argument("--dataset",type=Path); p.add_argument("--output",type=Path,required=True); p.add_argument("--device",default="cpu"); a=p.parse_args()
    import torch
    from torch.utils.data import DataLoader
    c=load_config(a.config); seed_everything(c["training"]["seed"]); root=a.dataset or Path(c["data"]["dataset"])
    training=create_dataset(root,"train",c,True); validation=create_dataset(root,"validation",c,False)
    generator=torch.Generator().manual_seed(c["training"]["seed"])
    loader=DataLoader(training,batch_size=c["training"]["batch_size"],shuffle=True,generator=generator,num_workers=c["data"]["workers"])
    val_loader=DataLoader(validation,batch_size=c["training"]["batch_size"],shuffle=False,num_workers=c["data"]["workers"])
    model=build_model(c).to(a.device); criterion=build_loss(c["training"]["class_weights"],c["training"]["dice_weight"])
    optimizer=torch.optim.AdamW(model.parameters(),lr=c["training"]["learning_rate"],weight_decay=c["training"]["weight_decay"])
    a.output.mkdir(parents=True,exist_ok=True); (a.output/"config.json").write_text(json.dumps(c,indent=2,sort_keys=True)+"\n")
    best=float("inf"); stale=0; history=[]
    for epoch in range(c["training"]["epochs"]):
        model.train(); train_loss=0.0
        for images,labels,_ in loader:
            images,labels=images.to(a.device),labels.to(a.device); optimizer.zero_grad(set_to_none=True)
            loss=criterion(model(images),labels); loss.backward(); optimizer.step(); train_loss+=loss.item()
        model.eval(); val_loss=0.0
        with torch.no_grad():
            for images,labels,_ in val_loader: val_loss+=criterion(model(images.to(a.device)),labels.to(a.device)).item()
        row={"epoch":epoch+1,"train_loss":train_loss/max(1,len(loader)),"validation_loss":val_loss/max(1,len(val_loader))}; history.append(row); print(json.dumps(row))
        if row["validation_loss"] < best:
            best=row["validation_loss"]; stale=0; torch.save({"state_dict":model.state_dict(),"config_sha256":config_sha256(c),"epoch":epoch+1},a.output/"best.pt")
        else: stale+=1
        if stale>=c["training"]["early_stopping_patience"]: break
    (a.output/"history.json").write_text(json.dumps(history,indent=2)+"\n")
if __name__=="__main__": main()
