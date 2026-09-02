#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""记录 controller_manager introspection 中手臂与腰部关节空间的接口数值。

数据源：/controller_manager/introspection_data/full
  (pal_statistics_msgs/msg/Statistics，controller_manager 每个周期发布)
  其中以 state_interface.<joint>/<field> / command_interface.<joint>/<field>
  的形式携带所有硬件接口的实时数值。本脚本过滤出手臂与腰部关节
  (left_jointN / right_jointN，随分体/双臂/全身模式自动变化；body_jointN
  为腰部，机器人没有 body 系统时自动为空)。

默认只记录 position（状态角 + 指令角）；需要 effort/velocity 时用
--fields 指定，如 --fields position,velocity,effort。

用法：
  python3 record_joint_interfaces.py                          # 只录 position, Ctrl+C 停
  python3 record_joint_interfaces.py -d 10                    # 录 10 秒
  python3 record_joint_interfaces.py --fields position,velocity,effort
  python3 record_joint_interfaces.py -d 10 --plot              # 录完生成交互式HTML图
  python3 record_joint_interfaces.py -o /tmp/rec

默认输出到脚本同目录下的 record_data/<YYYYMMDD_HHMMSS>/，每次录制创建
独立会话目录，不受启动命令所在目录影响。显式使用 -o 时直接使用指定目录。
"""
import argparse
import csv
import os
import re
import signal
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from pal_statistics_msgs.msg import Statistics

# state_interface.left_joint3/velocity、command_interface.body_joint1/position 等；
# 排除 xxx.is_limited 附加项。字段再由 --fields 过滤。
IFACE_RE = re.compile(
    r"^(state|command)_interface\.((?:left|right|body)_joint\d+)"
    r"/(position|velocity|effort)$")

VALID_FIELDS = ("position", "velocity", "effort")

INTROSPECTION_TOPIC = "/controller_manager/introspection_data/full"


def make_session_dir(record_root: str) -> str:
    """创建并返回带时间戳的会话目录，同秒冲突时追加 _2、_3。"""
    os.makedirs(record_root, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    suffix = 1
    while True:
        name = stamp if suffix == 1 else f"{stamp}_{suffix}"
        session_dir = os.path.join(record_root, name)
        try:
            os.mkdir(session_dir)
            return session_dir
        except FileExistsError:
            suffix += 1


class IfaceRecorder(Node):
    def __init__(self, out_dir: str, fields, do_plot: bool = False):
        super().__init__("joint_iface_recorder")
        os.makedirs(out_dir, exist_ok=True)
        self.out_dir = out_dir
        self.fields = fields
        self.do_plot = do_plot
        self.t0 = time.time()
        self.rows_buf = []        # 绘图用内存缓存：(elapsed, [列值...])
        self.rows = 0
        self.columns = []          # 保持首条消息的顺序，新出现的名字追加到末尾
        self.path = os.path.join(out_dir, "joint_interfaces.csv")
        self.f = open(self.path, "w", newline="")
        self.w = csv.writer(self.f)
        self.create_subscription(Statistics, INTROSPECTION_TOPIC,
                                  self.cb, qos_profile_sensor_data)
        self.get_logger().info(f"订阅 {INTROSPECTION_TOPIC} -> {self.path}")

    def cb(self, msg: Statistics):
        values = {}
        for stat in msg.statistics:
            m = IFACE_RE.match(stat.name)
            if m and m.group(3) in self.fields:
                values[m.group(0)] = stat.value
        if not values:
            return
        # 表头（首条消息写出；之后若接口集合变化则动态补列）
        new_cols = [n for n in values if n not in self.columns]
        if new_cols:
            self.columns.extend(new_cols)
            # 旧行补 nan，保持 rows_buf 每行长度与 columns 一致
            for row in self.rows_buf:
                row.extend([float("nan")] * len(new_cols))
            self.w.writerow(["stamp", "elapsed_s"] + self.columns)
            self.f.flush()
            # 关节数校验：手臂分体=7、双臂/全身=14；腰部 body_joint1~4，机器人
            # 没有 body 系统时为 0（head/工具关节均不应匹配）。不符说明命名约定
            # 变了，此时 CSV 内容未必是手臂/腰部，需人工核对。
            joints = sorted({c.split(".")[1].rsplit("/", 1)[0] for c in self.columns})
            arm = [j for j in joints if j.startswith(("left_", "right_"))]
            waist = [j for j in joints if j.startswith("body_")]
            if len(arm) not in (7, 14) or len(waist) not in (0, 4):
                self.get_logger().warn(
                    f"匹配到手臂 {len(arm)} 个(预期 分体7/双臂14)、"
                    f"腰部 {len(waist)} 个(预期 0/4)，与预期不符: {joints}")
            else:
                self.get_logger().info(
                    f"记录手臂关节 {len(arm)} 个: {arm}；"
                    f"腰部关节 {len(waist)} 个: {waist}")
        elapsed = time.time() - self.t0
        self.w.writerow(
            [f"{msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9:.9f}",
             f"{elapsed:.6f}"]
            + [values.get(c, "") for c in self.columns])
        # 行内首元素为 elapsed_s，其后与 self.columns 一一对应
        self.rows_buf.append([elapsed]
                             + [float(v) if v != "" else float("nan")
                                for v in (values.get(c, "") for c in self.columns)])
        self.rows += 1
        if self.rows % 200 == 0:
            self.f.flush()

    def close(self):
        self.f.flush()
        self.f.close()
        self.get_logger().info(f"已写入 {self.rows} 条、{len(self.columns)} 列 -> {self.path}")
        if self.do_plot:
            self.plot()

    def plot(self):
        """每个字段生成一个零依赖可交互 HTML（缩放/平移/悬停读数/关节筛选）。
        数据内嵌文件内，离线可用；长录制自动抽稀到每序列 ≤12000 点。"""
        if not self.rows_buf:
            self.get_logger().warn("无数据可绘图")
            return
        import json
        for field in sorted(self.fields):
            state_cols = {c.split(".")[1].rsplit("/", 1)[0]: i + 1
                          for i, c in enumerate(self.columns)
                          if c.startswith("state_interface.")
                          and c.rsplit("/", 1)[1] == field}
            cmd_cols = {c.split(".")[1].rsplit("/", 1)[0]: i + 1
                        for i, c in enumerate(self.columns)
                        if c.startswith("command_interface.")
                        and c.rsplit("/", 1)[1] == field}
            if not state_cols and not cmd_cols:
                continue
            joints = sorted(set(state_cols) | set(cmd_cols))
            rows = self.rows_buf
            n = len(rows)
            stride = max(1, (n + 11999) // 12000)
            payload = {
                "field": field,
                "t": [round(rows[i][0], 6) for i in range(0, n, stride)],
                "joints": joints,
                "state": {j: [rows[i][state_cols[j]] for i in range(0, n, stride)]
                          for j in joints if j in state_cols},
                "cmd": {j: [rows[i][cmd_cols[j]] for i in range(0, n, stride)]
                        for j in joints if j in cmd_cols},
            }
            path = os.path.join(self.out_dir, f"joint_interfaces_{field}.html")
            with open(path, "w", encoding="utf-8") as f:
                f.write(PLOT_TEMPLATE.replace("__DATA__", json.dumps(payload)))
            self.get_logger().info(f"绘图 -> {path}")


# 交互图表模板：__DATA__ 处注入 JSON。功能：拖框缩放 / 双击复位 /
# 滚轮平移 / 悬停十字线读数 / 关节勾选 / state-cmd-误差 视图切换。
PLOT_TEMPLATE = r"""<!DOCTYPE html>
<html lang="zh"><head><meta charset="utf-8">
<title>joint interfaces</title>
<style>
 body { margin:0; font:13px sans-serif; background:#1e1e1e; color:#ddd; }
 #bar { padding:8px 12px; display:flex; flex-wrap:wrap; gap:10px; align-items:center; }
 #bar label { cursor:pointer; user-select:none; }
 #chart { width:100%; height:calc(100vh - 96px); display:block; }
 #tip { position:fixed; pointer-events:none; background:#000c; padding:6px 8px;
        border-radius:4px; font:12px monospace; white-space:pre; display:none; }
 .grp { padding:4px 8px; border:1px solid #444; border-radius:4px; }
</style></head><body>
<div id="bar">
 <span class="grp" id="metric">
  <label><input type="radio" name="m" value="state" checked>state</label>
  <label><input type="radio" name="m" value="cmd">cmd</label>
  <label><input type="radio" name="m" value="err">cmd-state</label>
 </span>
 <span class="grp" id="joints"></span>
 <span style="color:#888">拖框缩放 · 双击复位 · 滚轮平移 · 悬停读数</span>
</div>
<canvas id="chart"></canvas>
<div id="tip"></div>
<script>
const D = __DATA__;
document.title = "joint " + D.field;
const cv = document.getElementById('chart'), ctx = cv.getContext('2d');
const tip = document.getElementById('tip');
const COLORS = ['#1f77b4','#ff7f0e','#2ca02c','#d62728','#9467bd','#8c564b',
                '#e377c2','#bcbd22','#17becf','#f39c12','#7f8c8d','#e74c3c',
                '#00b894','#fd79a8','#6c5ce7','#00cec9','#dfe6e9','#a29bfe'];
let x0 = D.t[0], x1 = D.t[D.t.length-1] || 1;
let metric = 'state';
let visible = new Set(D.joints);

function layout(){ const r = cv.getBoundingClientRect();
  cv.width = r.width * devicePixelRatio; cv.height = r.height * devicePixelRatio; draw(); }
addEventListener('resize', layout);

function val(j, i){
  if (metric === 'state') return D.state[j] ? D.state[j][i] : NaN;
  if (metric === 'cmd')   return D.cmd[j]   ? D.cmd[j][i]   : NaN;
  return (D.cmd[j] && D.state[j]) ? (D.cmd[j][i] - D.state[j][i]) : NaN;
}
function tIndex(t){ // 二分：最后一个 <= t 的下标
  let lo = 0, hi = D.t.length - 1;
  while (lo < hi){ const mid = (lo + hi + 1) >> 1; if (D.t[mid] <= t) lo = mid; else hi = mid - 1; }
  return lo;
}
function niceTicks(a, b, n){
  const step0 = (b - a) / n, mag = Math.pow(10, Math.floor(Math.log10(step0)));
  let step = mag; for (const m of [1,2,5,10]) if (step0 <= m*mag){ step = m*mag; break; }
  const out = []; for (let v = Math.ceil(a/step)*step; v <= b + 1e-12; v += step) out.push(v);
  return out;
}
function draw(){
  ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);
  ctx.clearRect(0,0,cv.width,cv.height);
  const W = cv.width/devicePixelRatio, H = cv.height/devicePixelRatio;
  const L = 64, R = 10, T = 10, B = 28;
  if (x1 - x0 < 1e-9) x1 = x0 + 1e-9;
  const i0 = tIndex(x0), i1 = tIndex(x1);
  let ymin = Infinity, ymax = -Infinity;
  for (const j of visible) for (let i = i0; i <= i1 && i < D.t.length; i++){
    const v = val(j, i);
    if (isFinite(v)){ if (v < ymin) ymin = v; if (v > ymax) ymax = v; }
  }
  if (!isFinite(ymin)){ ymin = -1; ymax = 1; }
  if (ymax - ymin < 1e-9){ ymin -= 0.5; ymax += 0.5; }
  const pad = (ymax - ymin) * 0.05; ymin -= pad; ymax += pad;
  const px = t => L + (t - x0) / (x1 - x0) * (W - L - R);
  const py = v => T + (ymax - v) / (ymax - ymin) * (H - T - B);
  // 网格与刻度
  ctx.strokeStyle = '#333'; ctx.fillStyle = '#888'; ctx.font = '11px monospace';
  for (const t of niceTicks(x0, x1, 8)){
    const X = px(t); if (X < L || X > W - R) continue;
    ctx.beginPath(); ctx.moveTo(X, T); ctx.lineTo(X, H - B); ctx.stroke();
    ctx.fillText(t.toFixed(2), X - 14, H - B + 16);
  }
  for (const v of niceTicks(ymin, ymax, 6)){
    const Y = py(v); if (Y < T || Y > H - B) continue;
    ctx.beginPath(); ctx.moveTo(L, Y); ctx.lineTo(W - R, Y); ctx.stroke();
    ctx.fillText(v.toPrecision(5), 4, Y + 4);
  }
  ctx.strokeStyle = '#666';
  ctx.strokeRect(L, T, W - L - R, H - T - B);
  // 曲线：当前 metric 实线；state 视图额外叠加 cmd 虚线便于对比
  function pathSeries(getv){
    ctx.beginPath(); let pen = false;
    for (let i = i0; i <= i1 && i < D.t.length; i++){
      const v = getv(i);
      if (!isFinite(v)){ pen = false; continue; }
      const X = px(D.t[i]), Y = py(v);
      pen ? ctx.lineTo(X, Y) : ctx.moveTo(X, Y); pen = true;
    }
    ctx.stroke();
  }
  let k = 0;
  for (const j of D.joints){
    if (visible.has(j)){
      ctx.strokeStyle = COLORS[k % COLORS.length]; ctx.lineWidth = 1.2;
      ctx.setLineDash([]); pathSeries(i => val(j, i));
      if (metric === 'state' && D.state[j] && D.cmd[j]){
        ctx.setLineDash([6,4]); pathSeries(i => D.cmd[j][i]);
      }
      ctx.setLineDash([]);
    }
    k++;
  }
  // 悬停十字线
  if (hoverI >= 0){
    const X = px(D.t[hoverI]);
    ctx.strokeStyle = '#fff5'; ctx.beginPath();
    ctx.moveTo(X, T); ctx.lineTo(X, H - B); ctx.stroke();
  }
}
let hoverI = -1, dragS = null;
cv.addEventListener('mousemove', e => {
  const r = cv.getBoundingClientRect();
  if (dragS){
    dragS.x1 = e.clientX - r.left; rubber(dragS); return;
  }
  const W = r.width, L = 64, R = 10;
  const t = x0 + (e.clientX - r.left - L) / (W - L - R) * (x1 - x0);
  hoverI = (t >= x0 && t <= x1) ? tIndex(t) : -1;
  draw();
  if (hoverI < 0){ tip.style.display = 'none'; return; }
  let s = 't=' + D.t[hoverI].toFixed(3) + 's\n';
  for (const j of D.joints) if (visible.has(j)){
    const st = D.state[j] ? D.state[j][hoverI] : NaN;
    const cm = D.cmd[j] ? D.cmd[j][hoverI] : NaN;
    s += j.padEnd(13) + ' st=' + (isFinite(st)?st.toFixed(4):'-')
       + ' cmd=' + (isFinite(cm)?cm.toFixed(4):'-')
       + ' err=' + (isFinite(st)&&isFinite(cm)?(cm-st).toFixed(4):'-') + '\n';
  }
  tip.textContent = s; tip.style.display = 'block';
  tip.style.left = (e.clientX + 16) + 'px'; tip.style.top = (e.clientY + 16) + 'px';
});
cv.addEventListener('mouseleave', () => { hoverI = -1; tip.style.display = 'none'; draw(); });
cv.addEventListener('mousedown', e => {
  const r = cv.getBoundingClientRect();
  dragS = { x0: e.clientX - r.left, x1: e.clientX - r.left };
});
addEventListener('mouseup', e => {
  if (!dragS) return;
  const r = cv.getBoundingClientRect(), W = r.width, L = 64, R = 10;
  const a = Math.min(dragS.x0, dragS.x1), b = Math.max(dragS.x0, dragS.x1);
  dragS = null; ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);
  if (b - a < 6){ draw(); return; }   // 点击，不缩放
  const tA = x0 + (a - L) / (W - L - R) * (x1 - x0);
  const tB = x0 + (b - L) / (W - L - R) * (x1 - x0);
  x0 = tA; x1 = tB; draw();
});
function rubber(s){
  draw();
  ctx.fillStyle = '#fff2';
  ctx.fillRect(Math.min(s.x0, s.x1), 10, Math.abs(s.x1 - s.x0), cv.height/devicePixelRatio - 38);
}
cv.addEventListener('dblclick', () => { x0 = D.t[0]; x1 = D.t[D.t.length-1]; draw(); });
cv.addEventListener('wheel', e => {
  e.preventDefault();
  const span = x1 - x0, d = (e.deltaY > 0 ? 1 : -1) * span * 0.1;
  x0 += d; x1 += d; draw();
}, { passive:false });
document.getElementById('metric').addEventListener('change', e => { metric = e.target.value; draw(); });
const jbox = document.getElementById('joints');
D.joints.forEach((j, k) => {
  const lb = document.createElement('label');
  lb.innerHTML = `<input type="checkbox" checked><span style="color:${COLORS[k % COLORS.length]}">■</span> ${j}`;
  lb.querySelector('input').addEventListener('change', e => {
    e.target.checked ? visible.add(j) : visible.delete(j); draw();
  });
  jbox.appendChild(lb);
});
layout();
</script></body></html>"""


def main():
    parser = argparse.ArgumentParser(
        description="记录手臂与腰部关节接口数值(controller_manager introspection)")
    record_root = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "record_data")
    parser.add_argument(
        "-o", "--output", default=None,
        help=("精确输出目录；不指定时自动创建时间会话目录 "
              f"{record_root}/<YYYYMMDD_HHMMSS>"),
    )
    parser.add_argument("-d", "--duration", type=float, default=0.0,
                        help="录制时长(秒)，0=Ctrl+C手动停止")
    parser.add_argument("--fields", default="position",
                        help="记录的接口字段，逗号分隔"
                             f"（可选: {','.join(VALID_FIELDS)}，默认仅 position）")
    parser.add_argument("--plot", action="store_true",
                        help="录制结束后生成可交互 HTML 图表(缩放/平移/悬停读数)")
    args = parser.parse_args()

    fields = {f.strip() for f in args.fields.split(",") if f.strip()}
    invalid = fields - set(VALID_FIELDS)
    if invalid:
        parser.error(f"非法字段: {','.join(invalid)}，可选: {','.join(VALID_FIELDS)}")

    node = None
    timer = None
    rclpy_initialized = False
    stop_requested = threading.Event()
    previous_signal_handlers = {}

    def request_stop(_signum, _frame):
        # 不在信号回调中调用 rclpy.shutdown()；让主循环退出后依次完成
        # CSV/HTML 落盘、节点销毁与 context shutdown。
        stop_requested.set()

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_signal_handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, request_stop)

        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        rclpy_initialized = True
        out_dir = args.output if args.output is not None else make_session_dir(record_root)
        node = IfaceRecorder(out_dir, fields, do_plot=args.plot)
        if args.duration > 0:
            timer = threading.Timer(args.duration, stop_requested.set)
            timer.start()
        while rclpy.ok() and not stop_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"recorder stopped: {exc}", file=sys.stderr)
        raise
    finally:
        if timer is not None:
            timer.cancel()
        try:
            if node is not None:
                try:
                    node.close()
                finally:
                    node.destroy_node()
        finally:
            if rclpy_initialized and rclpy.ok():
                rclpy.shutdown()
            for signum, previous_handler in previous_signal_handlers.items():
                signal.signal(signum, previous_handler)


if __name__ == "__main__":
    main()
