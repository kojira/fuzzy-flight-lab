import {WORLD,INITIAL,LABELS,Evolution,control,episode,evaluate,clamp} from './core.js';
const $=id=>document.getElementById(id);
const canvas=$('world'),ctx=canvas.getContext('2d'),chart=$('chart'),cc=chart.getContext('2d');
let evo,obstacles,playing=false,learning=false,flight,flightGenome,flightGeneration,index=0,lastFrame=0,selected=-1,drag=null,changed=false;
const copyLayout=()=>INITIAL.map(o=>({...o}));
const statusText={running:'飛行中',goal:'ゴール到達！',collision:'衝突',timeout:'時間切れ'};
const bars=(id,labels)=>$(id).innerHTML=labels.map((s,i)=>`<div class="bar-row"><label>${s}</label><div class="bar-track"><div class="bar-fill" id="${id}-${i}"></div></div><output id="${id}-n-${i}">0.00</output></div>`).join('');
bars('near-bars',['左前','正面','右前']);bars('goal-bars',['左','正面','右']);
function updateBars(id,values){values.forEach((v,i)=>{$(`${id}-${i}`).style.width=`${clamp(v)*100}%`;$(`${id}-n-${i}`).textContent=v.toFixed(2);});}
function updateTree(s){const c=control(s,obstacles,flightGenome);updateBars('near-bars',c.memberships);updateBars('goal-bars',[c.goalLeft,c.aligned,c.goalRight]);$('distance').textContent=`距離：左前 ${c.distance[0].toFixed(0)} / 正面 ${c.distance[1].toFixed(0)} / 右前 ${c.distance[2].toFixed(0)} px`;
  for(const [id,key] of [['avoid','avoidance'],['danger','danger'],['attract','attraction'],['weight','weight'],['turn','turn'],['speed','speed']])$(id).textContent=c[key].toFixed(2);
  return c;
}
function drawChart(){
  const w=chart.width,h=chart.height;cc.clearRect(0,0,w,h);const values=evo.history.map(p=>p.score),lo=Math.min(...values)-5,hi=Math.max(...values)+5;
  const y=v=>h-25-(v-lo)/(hi-lo)*(h-45);cc.font='11px system-ui';cc.fillStyle='#8faac3';cc.strokeStyle='#21364c';
  for(let i=0;i<3;i++){const v=lo+(hi-lo)*i/2;cc.beginPath();cc.moveTo(45,y(v));cc.lineTo(w-15,y(v));cc.stroke();cc.fillText(v.toFixed(0),5,y(v)+4);}
  cc.strokeStyle='#55e1d6';cc.lineWidth=2;cc.beginPath();evo.history.forEach((p,i)=>{const x=45+i/Math.max(1,values.length-1)*(w-65);i?cc.lineTo(x,y(p.score)):cc.moveTo(x,y(p.score));});cc.stroke();
  const end=evo.history.at(-1),x=45+(values.length>1?w-65:0);cc.fillStyle='#55e1d6';cc.beginPath();cc.arc(x,y(end.score),4,0,Math.PI*2);cc.fill();cc.fillStyle='#8faac3';cc.fillText('世代 0',45,h-5);cc.fillText(`世代 ${evo.generation}`,w-80,h-5);
}
function metrics(){
  const a=evaluate(evo.best,INITIAL),b=evaluate(evo.best,obstacles);
  $('generation').textContent=evo.generation;
  for(const [prefix,r] of [['train',a],['eval',b]]){$(`${prefix}-score`).textContent=`${r.goals} / 3 到達`;$(`${prefix}-detail`).textContent=`平均 ${r.score.toFixed(1)} 点 · 衝突 ${r.collisions} 回`;}
  $('genes').innerHTML=evo.best.map((v,i)=>`<span>${LABELS[i]}：${v.toFixed(2)}</span>`).join('');
  $('layout-state').textContent=changed?'変更した評価配置（学習には使いません）':'学習と同じ配置';
  drawChart();
}
function replay(){flightGenome=evo.best.slice();flightGeneration=evo.generation;flight=episode(flightGenome,obstacles,0,true);index=0;playing=true;lastFrame=0;}
function setLearning(value){learning=value;$('train').textContent=value?'■ 学習ストップ':'▶ 学習スタート';$('once').disabled=value;$('activity').textContent=value?'進化中：固定した学習配置で選択・交叉・突然変異を実行。いつでも止められます。':`停止中：世代 ${evo.generation}。障害物を動かして、未知配置を試そう。`;}
function reset(){evo=new Evolution($('seed').value||'ALPHA-2016');setLearning(false);obstacles=copyLayout();changed=false;selected=-1;metrics();replay();$('activity').textContent='seedから初期集団を再生成しました。世代0の実測成績です。';}
$('train').addEventListener('click',()=>setLearning(!learning));
$('once').addEventListener('click',()=>{evo.advance();metrics();replay();$('activity').textContent=`世代 ${evo.generation} を評価しました。実測値を更新。`;});
$('reset').addEventListener('click',reset);$('replay').addEventListener('click',replay);
$('restore').addEventListener('click',()=>{obstacles=copyLayout();changed=false;selected=-1;metrics();replay();});
function position(e){const r=canvas.getBoundingClientRect();return {x:(e.clientX-r.left)*WORLD.w/r.width,y:(e.clientY-r.top)*WORLD.h/r.height};}
function moveObstacle(i,p){const o=obstacles[i];o.x=clamp(p.x,o.r+12,WORLD.w-o.r-12);o.y=clamp(p.y,o.r+12,WORLD.h-o.r-12);changed=obstacles.some((v,k)=>v.x!==INITIAL[k].x||v.y!==INITIAL[k].y);}
canvas.addEventListener('pointerdown',e=>{e.preventDefault();canvas.focus();const p=position(e),hit=obstacles.findIndex(o=>Math.hypot(p.x-o.x,p.y-o.y)<=o.r+12);canvas.setPointerCapture(e.pointerId);if(hit>=0){selected=hit;drag={...p,ox:obstacles[hit].x-p.x,oy:obstacles[hit].y-p.y,moved:false};}else if(selected>=0){moveObstacle(selected,p);metrics();replay();}else{selected=-1;}});
canvas.addEventListener('pointermove',e=>{if(!drag)return;const p=position(e);if(Math.hypot(p.x-drag.x,p.y-drag.y)>3)drag.moved=true;if(drag.moved)moveObstacle(selected,{x:p.x+drag.ox,y:p.y+drag.oy});});
function endDrag(){if(drag?.moved){metrics();replay();}drag=null;}
canvas.addEventListener('pointerup',endDrag);canvas.addEventListener('pointercancel',endDrag);
canvas.addEventListener('keydown',e=>{const delta={ArrowLeft:[-10,0],ArrowRight:[10,0],ArrowUp:[0,-10],ArrowDown:[0,10]}[e.key];if(delta&&selected>=0){e.preventDefault();moveObstacle(selected,{x:obstacles[selected].x+delta[0],y:obstacles[selected].y+delta[1]});metrics();replay();}});
function draw(){
  const s=flight.path[Math.min(index,flight.path.length-1)];ctx.clearRect(0,0,WORLD.w,WORLD.h);
  ctx.strokeStyle='#20334a';ctx.lineWidth=1;for(let x=0;x<=900;x+=45){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,540);ctx.stroke();}for(let y=0;y<=540;y+=45){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(900,y);ctx.stroke();}
  ctx.lineWidth=2;ctx.strokeStyle='#65845b';ctx.setLineDash([5,6]);ctx.beginPath();ctx.arc(WORLD.goal.x,WORLD.goal.y,35,0,Math.PI*2);ctx.stroke();ctx.setLineDash([]);ctx.fillStyle='#7de59b30';ctx.beginPath();ctx.arc(WORLD.goal.x,WORLD.goal.y,WORLD.goal.r,0,Math.PI*2);ctx.fill();ctx.fillStyle='#b1efb9';ctx.font='bold 12px system-ui';ctx.fillText('GOAL',WORLD.goal.x-17,WORLD.goal.y+4);ctx.fillStyle='#819bb7';ctx.fillText('START',35,320);
  if(changed){ctx.strokeStyle='#3e5a73';ctx.setLineDash([4,6]);for(const o of INITIAL){ctx.beginPath();ctx.arc(o.x,o.y,o.r,0,Math.PI*2);ctx.stroke();}ctx.setLineDash([]);}
  obstacles.forEach((o,i)=>{ctx.fillStyle=i===selected?'#b87c493a':'#ffac701d';ctx.strokeStyle=i===selected?'#ffd2a8':'#e4a16b';ctx.lineWidth=i===selected?3:1.5;ctx.beginPath();ctx.arc(o.x,o.y,o.r,0,Math.PI*2);ctx.fill();ctx.stroke();ctx.fillStyle='#d8aa88';ctx.font='12px system-ui';ctx.fillText(String(i+1),o.x-3,o.y+4);});
  ctx.strokeStyle='#55e1d696';ctx.lineWidth=2;ctx.beginPath();for(let i=0;i<=index&&i<flight.path.length;i++){const p=flight.path[i];i?ctx.lineTo(p.x,p.y):ctx.moveTo(p.x,p.y);}ctx.stroke();
  const c=updateTree(s);if(!drag){c.distance.forEach((d,i)=>{const a=s.heading+(i-1)*flightGenome[4];ctx.strokeStyle=`rgba(85,225,214,${.15+c.memberships[i]*.6})`;ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(s.x,s.y);ctx.lineTo(s.x+Math.cos(a)*d,s.y+Math.sin(a)*d);ctx.stroke();});}
  ctx.save();ctx.translate(s.x,s.y);ctx.rotate(s.heading);ctx.fillStyle=s.status==='collision'?'#ff867d':'#55e1d6';ctx.shadowBlur=14;ctx.shadowColor='#55e1d6';ctx.beginPath();ctx.moveTo(15,0);ctx.lineTo(-10,-9);ctx.lineTo(-6,0);ctx.lineTo(-10,9);ctx.closePath();ctx.fill();ctx.restore();
  $('flight-status').textContent=`世代${flightGeneration} · ${statusText[s.status]} · ${s.steps} step`;
}
let lastGeneration=0;
function frame(t){
  if(learning&&!drag&&t-lastGeneration>150){lastGeneration=t;evo.advance();metrics();if(!playing)replay();}
  if(playing&&(!lastFrame||t-lastFrame>=25)&&!drag){index=Math.min(index+2,flight.path.length-1);lastFrame=t;if(index>=flight.path.length-1){playing=false;if(flightGeneration!==evo.generation)replay();}}
  draw();requestAnimationFrame(frame);
}
// Read-only snapshot for automated checks; no control bypass.
Object.defineProperty(window,'labSnapshot',{get:()=>({generation:evo.generation,learning,changed,obstacles:obstacles.map(o=>({...o})),best:evo.best.slice(),history:evo.history.map(p=>({...p})),flightStatus:flight.status,flightGeneration,flightGenome:flightGenome.slice(),flightStep:flight.path[Math.min(index,flight.path.length-1)].steps,flightState:{...flight.path[Math.min(index,flight.path.length-1)]}})});
reset();requestAnimationFrame(frame);
