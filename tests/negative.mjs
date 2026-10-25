// Deliberately break each regression, verify it fails, always restore the source.
import {readFile,writeFile} from 'node:fs/promises';
import {spawnSync} from 'node:child_process';
import assert from 'node:assert/strict';
const cases=[
 {name:'nn-weighted-sum',file:'core.js',from:'sum+=x*g[k++]',to:'sum+=0*g[k++]',command:['--test','--test-name-pattern=NN zero/explicit','tests/core.test.mjs']},
 {name:'method-state-leak',file:'core.js',from:"return this.states[method]??(this.states[method]=new Evolution('ALPHA-2016',24,method));",to:"return this.states[method]=new Evolution('ALPHA-2016',24,method);",command:['--test','--test-name-pattern=method switch','tests/core.test.mjs']},
 {name:'nn-step-dispatch',file:'core.js',from:'control(s,obstacles,g,method),heading',to:"control(s,obstacles,g,'gft'),heading",command:['--test','--test-name-pattern=NN physical','tests/core.test.mjs']},
 {name:'nn-live-output',file:'app.js',from:"$('nn-turn').textContent=c.turn.toFixed(2)",to:"$('nn-turn').textContent='999.00'",command:['tests/browser-nn.mjs']},
 {name:'nn-reset-other-state',file:'core.js',from:'reset(seed){return this.states[this.method]=',to:"reset(seed){delete this.states[this.method==='nn'?'gft':'nn'];return this.states[this.method]=",command:['--test','--test-name-pattern=method switch','tests/core.test.mjs']},
];
const results=[];
for(const c of cases){const original=await readFile(c.file,'utf8');assert.ok(original.includes(c.from),c.name);try{await writeFile(c.file,original.replace(c.from,c.to));const r=spawnSync(process.execPath,c.command,{encoding:'utf8',env:process.env,timeout:60000});await writeFile(`artifacts/nn-negative-${c.name}.txt`,r.stdout+'\n'+r.stderr);assert.notEqual(r.status,0,`${c.name}: negative unexpectedly passed`);assert.equal(r.error,undefined,`${c.name}: command error`);results.push({name:c.name,exitCode:r.status,detected:true});}finally{await writeFile(c.file,original);}}
await writeFile('artifacts/nn-negative-results.json',JSON.stringify(results,null,2));console.log(JSON.stringify(results));
