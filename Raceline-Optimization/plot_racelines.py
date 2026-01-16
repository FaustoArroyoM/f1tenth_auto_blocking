#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import argparse

import numpy as np
import pandas as pd
import yaml
from PIL import Image
import matplotlib.pyplot as plt

# =========================
# VIEW WINDOW (world coords)
# =========================
X_MIN,X_MAX=-4.5,5.5
Y_MIN,Y_MAX=-2.0,6.0

# plot Racelines that go inside on the corners
all_inside = True

def read_xy_csv(path: Path) -> pd.DataFrame:
    df=pd.read_csv(path)
    # tolerate common headers + "no header"
    xcol=next((c for c in ["x","x_m","# x_m","#x_m","# x","#x"] if c in df.columns),None)
    ycol=next((c for c in ["y","y_m","# y_m","#y_m","# y","#y"] if c in df.columns),None)
    if xcol is None or ycol is None:
        df=pd.read_csv(path,header=None)
        if df.shape[1]<2:
            raise ValueError(f"{path} has <2 columns")
        df=df.rename(columns={0:"x",1:"y"})
    else:
        df=df.rename(columns={xcol:"x",ycol:"y"})
    df["x"]=pd.to_numeric(df["x"],errors="coerce")
    df["y"]=pd.to_numeric(df["y"],errors="coerce")
    df=df.dropna(subset=["x","y"])
    if len(df)<5:
        raise ValueError(f"{path}: too few valid points (n={len(df)})")
    return df[["x","y"]]


def read_and_crop_map(map_yaml: Path,map_pgm: Path):
    cfg=yaml.safe_load(map_yaml.read_text())
    res=float(cfg["resolution"])
    ox,oy=float(cfg["origin"][0]),float(cfg["origin"][1])

    img=np.array(Image.open(map_pgm))
    img=np.flipud(img)  # world y up
    h,w=img.shape

    wx2px=lambda x:int(np.floor((x-ox)/res))
    wy2py=lambda y:int(np.floor((y-oy)/res))

    px0=np.clip(wx2px(X_MIN),0,w); px1=np.clip(wx2px(X_MAX),0,w)
    py0=np.clip(wy2py(Y_MIN),0,h); py1=np.clip(wy2py(Y_MAX),0,h)
    if px1<=px0 or py1<=py0:
        raise ValueError("Crop window does not overlap map (check origin/resolution/window).")

    cropped=img[py0:py1,px0:px1]
    extent=[ox+px0*res,ox+px1*res,oy+py0*res,oy+py1*res]
    return cropped,extent


def offsets_closed_loop(x: np.ndarray,y: np.ndarray,half_w: float):
    dx=np.roll(x,-1)-np.roll(x,1)
    dy=np.roll(y,-1)-np.roll(y,1)
    n=np.hypot(dx,dy); n[n<1e-12]=1e-12
    tx,ty=dx/n,dy/n
    nx,ny=-ty,tx  # left normal
    return x+half_w*nx,y+half_w*ny,x-half_w*nx,y-half_w*ny


def unit_tangent(x: np.ndarray,y: np.ndarray,i0: int=0):
    n=len(x)
    for k in (1,2,3):
        i1=(i0+k)%n
        dx=float(x[i1]-x[i0]); dy=float(y[i1]-y[i0])
        L=(dx*dx+dy*dy)**0.5
        if L>1e-12:
            return dx/L,dy/L
    return 1.0,0.0


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--map-yaml",type=Path,default=Path("maps/FTM_clean2_ws25.yaml"))
    ap.add_argument("--map-pgm",type=Path,default=Path("maps/FTM_clean2_ws25.pgm"))
    if all_inside:
        ap.add_argument("--inside",type=Path,default=Path("outputs/3 Racelines all inside/Raceline_inside.csv"))
        ap.add_argument("--middle",type=Path,default=Path("outputs/3 Racelines all inside/Raceline_middle.csv"))
        ap.add_argument("--outside",type=Path,default=Path("outputs/3 Racelines all inside/Raceline_outside.csv"))
    else:
        ap.add_argument("--inside",type=Path,default=Path("outputs/3 Racelines/Raceline_inside.csv"))
        ap.add_argument("--middle",type=Path,default=Path("outputs/3 Racelines/Raceline_middle.csv"))
        ap.add_argument("--outside",type=Path,default=Path("outputs/3 Racelines/Raceline_outside.csv"))

    # ✅ FIX: reftrack lives in inputs/tracks
    ap.add_argument("--reftrack",type=Path,default=Path("inputs/tracks/FTM_clean2_ws25.csv"))

    ap.add_argument("--vehicle-width",type=float,default=0.296)
    ap.add_argument("--dash-lw",type=float,default=0.6)
    ap.add_argument("--ref-lw",type=float,default=1.1)
    ap.add_argument("--start-arrow-len",type=float,default=1.0)
    args=ap.parse_args()

    if not args.reftrack.exists():
        raise FileNotFoundError(f"Reftrack not found: {args.reftrack.resolve()} (pass --reftrack <path>)")

    map_img,extent=read_and_crop_map(args.map_yaml,args.map_pgm)
    ref=read_xy_csv(args.reftrack)
    inside=read_xy_csv(args.inside)
    middle=read_xy_csv(args.middle)
    outside=read_xy_csv(args.outside)

    fig,ax=plt.subplots(figsize=(10,6))
    ax.imshow(map_img,cmap="gray",origin="lower",extent=extent,zorder=0)

    # reftrack BEHIND racelines
    ax.plot(ref["x"],ref["y"],color="0.25",alpha=0.9,
            linewidth=0.5,label="centerline",zorder=2)

    def plot_rl(df,color,label):
        x=df["x"].to_numpy(float); y=df["y"].to_numpy(float)
        xL,yL,xR,yR=offsets_closed_loop(x,y,0.5*args.vehicle_width)
        ax.plot(x,y,color=color,linewidth=2.2,label=label,zorder=6)
        ax.plot(xL,yL,"--",color=color,linewidth=0.4,alpha=0.9,zorder=7)
        ax.plot(xR,yR,"--",color=color,linewidth=0.4,alpha=0.9,zorder=7)

    plot_rl(inside,"tab:green","inside")
    plot_rl(middle,"tab:red","middle")
    plot_rl(outside,"tab:blue","outside")

    # start arrow: tail EXACTLY at ref start point
    xr=ref["x"].to_numpy(float); yr=ref["y"].to_numpy(float)
    x0,y0=float(xr[0]),float(yr[0])
    vx,vy=float(xr[20]-xr[0]),float(yr[20]-yr[0])
    L=(vx*vx+vy*vy)**0.5; vx,vy=(vx/L,vy/L) if L>1e-12 else (1.0,0.0)
    dx,dy=args.start_arrow_len*vx,args.start_arrow_len*vy
    ax.annotate("",xy=(x0+dx,y0+dy),xytext=(x0,y0),
                arrowprops=dict(arrowstyle="-|>",lw=2.2,color="tab:green",
                                mutation_scale=54,shrinkA=0,shrinkB=0),
                zorder=1)

    ax.set_xlim(X_MIN,X_MAX); ax.set_ylim(Y_MIN,Y_MAX)
    ax.set_xlabel("east in m"); ax.set_ylabel("north in m")
    ax.set_aspect("equal",adjustable="box")
    ax.legend()
    fig.tight_layout()
    plt.show()


if __name__=="__main__":
    main()
