#!/usr/bin/env bash
# =====================================================================
# disable_cpu_isolation.sh  (v3 - 已在 RK3588(TER30R) 真机验证)
# 取消内核 CPU 隔离: isolcpus / nohz_full / rcu_nocbs
#
# 原理(关键):
#   boot 分区(mmcblk0p3)是带 sha256 校验的 FIT 镜像, 内含 fdt/kernel/
#   resource 三个子镜像。内核真正用的 cmdline 在 resource 的 rk-kernel.dtb
#   里(不是 fdt)。本脚本: 在 fdt+resource 数据区把隔离参数等长替换为空格,
#   再【重算这两个子镜像的 sha256 并写回】, 使 FIT 自洽; FIT 的 RSA 签名虽
#   变旧但该机型不强制(uboot 无 required), 故可正常启动。
#   全程纯 Python, 不需要联网/mkimage/dtc, 也不需要私钥。
#
# 安全设计:
#   - 先整盘备份 p3; 写盘前做多重自校验, 任一不过即中止, 绝不写坏图。
#   - 检测到 uboot 强制验签(required)直接中止(那种机型改了必砖)。
#   - 默认交互确认; 加 -y 跳过(用于批量)。
#
# !! 刷其它机器前必看 !!
#   万一某台不开机, 用 RKDevTool 的"下载镜像"只刷 boot 行, 镜像用同型号
#   原版 boot(如 boot_GOOD_from_20.img, 同型号通用)即可救回。第一次刷新
#   机型时, 建议先把该机 p3 备份拷到电脑留底。
#
# 用法:  sudo ./disable_cpu_isolation.sh         # 交互确认
#        sudo ./disable_cpu_isolation.sh -y      # 自动确认(批量)
# =====================================================================
set -euo pipefail

PART=/dev/mmcblk0p3
UBOOT=/dev/mmcblk0p1
WORK=/home/lab
TS=$(date +%Y%m%d_%H%M%S)
BACKUP=$WORK/boot_p3_backup_$TS.img
CUR=$WORK/.boot_cur_$TS.img
NEW=$WORK/.boot_new_$TS.img
KEYS="isolcpus nohz_full rcu_nocbs"     # 要清除的隔离参数

die(){ echo "  !! $*" >&2; exit 1; }
[ "$(id -u)" = 0 ] || exec sudo -n bash "$0" "$@"

echo "[1/6] 安全检查: uboot 是否强制验签"
REQ=$(dd if=$UBOOT bs=1M status=none 2>/dev/null | grep -ac "required" || true)
[ "${REQ:-0}" = 0 ] || die "检测到 uboot 含 'required'(可能强制验签), 改 boot 会变砖 -> 中止"
echo "      OK: 未发现强制验签"

echo "[2/6] 备份 boot 分区 -> $BACKUP"
dd if=$PART of=$BACKUP bs=1M conv=fsync status=none
sha256sum "$BACKUP" | sed 's/^/      sha256: /'

echo "[3/6] 读取分区到工作副本"
dd if=$PART of=$CUR bs=1M status=none

echo "[4/6] 改 cmdline + 重算 hash + 自校验(不写盘)"
python3 - "$CUR" "$NEW" $KEYS <<'PY' || die "处理或自校验失败, 未写盘"
import sys,struct,hashlib,zlib
src,dst=sys.argv[1],sys.argv[2]; keys=[k.encode() for k in sys.argv[3:]]
TARGETS={"fdt","resource"}          # 只动这两个; kernel 内置串保持不动
b=bytearray(open(src,"rb").read())
m,total,os_,ostr,_,_,_,_,szs,szt=struct.unpack(">10I",b[:40])
if m!=0xd00dfeed: sys.exit("  不是 FIT 镜像")
def cs(o):
    e=b.index(b"\0",o); return b[o:e].decode("ascii","replace")
# 解析结构, 记录 path -> {prop:(val_off,len)} 和子节点
i=os_; path=[]; props={}
while i<os_+szt:
    tag=struct.unpack(">I",b[i:i+4])[0]; i+=4
    if tag==1:
        nm=cs(i); i+=((len(nm)+1+3)//4)*4; path.append(nm)
        props.setdefault("/".join(path),{})
    elif tag==2: path.pop()
    elif tag==3:
        ln,no=struct.unpack(">II",b[i:i+8]); i+=8
        props.setdefault("/".join(path),{})[cs(ostr+no)]=(i,ln); i+=((ln+3)//4)*4
    elif tag==9: break
def gv(p,n):
    o,l=props[p][n]; return b[o:o+l],o,l
def u32(v): return struct.unpack(">I",v)[0]
def region_of(p):
    pr=props[p]
    if "data-position" in pr and "data-size" in pr:
        return u32(gv(p,"data-position")[0]), u32(gv(p,"data-size")[0])
    if "data" in pr:
        o,l=pr["data"]; return o,l
    return None
def calc(algo,data):
    if algo=="sha256": return hashlib.sha256(data).digest()
    if algo=="sha1":   return hashlib.sha1(data).digest()
    if algo=="md5":    return hashlib.md5(data).digest()
    if algo=="crc32":  return struct.pack(">I",zlib.crc32(data)&0xffffffff)
    return None
def hash_nodes(imgpath):
    out=[]
    for q in props:
        if q.startswith(imgpath+"/") and q.split("/")[-1].startswith("hash") \
           and "algo" in props[q] and "value" in props[q]:
            out.append(q)
    return out

images=[p for p in props if p.startswith("/images/") and p.count("/")==2 and region_of(p)]
modified=set()
for p in images:
    name=p.split("/")[-1]
    if name not in TARGETS: continue
    dp,ds=region_of(p); reg=b[dp:dp+ds]; n=0
    for k in keys:
        j=0
        while True:
            x=reg.find(k,j)
            if x<0: break
            e=x+len(k)
            if reg[e:e+1]==b"=":
                t=e
                while t<len(reg) and reg[t] not in (0x20,0x00): t+=1
                print("  [%s] 清除 @+0x%x: %s"%(name,x,bytes(reg[x:t]).decode("ascii","replace")))
                for y in range(x,t): reg[y]=0x20
                n+=1
            j=e
    if n:
        b[dp:dp+ds]=reg; modified.add(p)
        for h in hash_nodes(p):
            algo=gv(h,"algo")[0].rstrip(b"\0").decode()
            vo,vl=props[h]["value"]
            nv=calc(algo,bytes(b[dp:dp+ds]))
            if nv is None or len(nv)!=vl: sys.exit("  不支持的 hash 或长度不符: %s/%s"%(name,algo))
            b[vo:vo+vl]=nv
            print("  [%s] 重算 %s hash"%(name,algo))
if not modified: sys.exit("  目标镜像里没找到隔离参数, 可能已处理或机型不同 -> 中止")

open(dst,"wb").write(b)
# ---- 自校验 ----
b2=open(dst,"rb").read(); ok=True
for p in images:
    name=p.split("/")[-1]; dp,ds=region_of(p)
    for h in hash_nodes(p):
        algo=gv(h,"algo")[0].rstrip(b"\0").decode()
        vo,vl=props[h]["value"]; stored=b2[vo:vo+vl]; c=calc(algo,b2[dp:dp+ds])
        good=(c==stored)
        if not good: ok=False
        print("  校验 %-9s %-6s %s"%(name,algo,"OK" if good else "BAD"))
    if name in TARGETS:
        if any((k+b"=") in b2[dp:dp+ds] for k in keys):
            ok=False; print("  校验 %-9s 仍含隔离参数 BAD"%name)
if len(b2)!=len(b): ok=False; print("  大小变化 BAD")
if b2[:4]!=bytes.fromhex("d00dfeed"): ok=False; print("  magic BAD")
print("=== 自校验:", "全部通过 ✓" if ok else "存在失败 ✗")
sys.exit(0 if ok else 1)
PY
prev=$(strings "$NEW" | grep -m1 "console=ttyFIQ0" || true)
echo "      新 cmdline 预览:"; echo "        $prev"

echo "[5/6] 写回 boot 分区"
if [ "${1:-}" = "-y" ] || [ "${1:-}" = "--yes" ]; then ans=yes
elif [ -t 0 ]; then read -r -p "      自校验已过, 确认写回 $PART ? 输入 yes: " ans
else die "非交互且未加 -y, 安全中止 (用: sudo $0 -y)"; fi
[ "$ans" = "yes" ] || { echo "已取消, 未写入"; rm -f "$CUR" "$NEW"; exit 0; }
dd if="$NEW" of="$PART" bs=1M conv=fsync status=none; sync
# 写后校验
A=$(dd if=$PART bs=1M count=96 status=none | sha256sum | cut -d' ' -f1)
B=$(sha256sum "$NEW" | cut -d' ' -f1)
[ "$A" = "$B" ] && echo "      写入校验一致 ✓" || die "写入校验不一致! 立即用备份恢复: dd if=$BACKUP of=$PART bs=1M conv=fsync"
rm -f "$CUR" "$NEW"

echo "[6/6] 完成。请重启生效:  sudo reboot"
echo "      重启后确认:        cat /proc/cmdline ; cat /sys/devices/system/cpu/isolated"
echo "      备份(可恢复):      $BACKUP"
echo "      回滚:              sudo dd if=$BACKUP of=$PART bs=1M conv=fsync"
