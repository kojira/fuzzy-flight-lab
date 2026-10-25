import {Evolution,evaluate,INITIAL,WORLD} from '../core.js';
import {writeFile} from 'node:fs/promises';
const seeds=['ALPHA-2016','test',...Array.from({length:18},(_,i)=>`flight-${i+1}`)];
const results=seeds.map(seed=>{const e=new Evolution(seed);const before=evaluate(e.initialBest,INITIAL);for(let i=0;i<35;i++)e.advance();const after=evaluate(e.best,INITIAL);return {seed,before,after,firstFullGeneration:e.history.find(x=>x.goals===3)?.generation??null,history:e.history};});
await writeFile(new URL('../artifacts/multi-seed.json',import.meta.url),JSON.stringify({conditions:{maxSteps:WORLD.maxSteps,population:24,generations:35,layout:INITIAL,headings:[-.18,0,.18],samePhysicsBeforeAfter:true},results},null,2));
console.log(JSON.stringify(results.map(r=>({seed:r.seed,beforeGoals:r.before.goals,afterGoals:r.after.goals,beforeScore:+r.before.score.toFixed(2),afterScore:+r.after.score.toFixed(2),firstFullGeneration:r.firstFullGeneration}))));
