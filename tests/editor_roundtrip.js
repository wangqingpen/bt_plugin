// Exercise the deployed editor's actual XML/graph helpers without a browser,
// HTTP requests, or any connection to a running gateway.
const fs = require('fs');
const path = require('path');
const assert = require('assert/strict');
const crypto = require('crypto');
const [sourcePath, treePath, modelsPath, outputDir] = process.argv.slice(2);
const source = fs.readFileSync(sourcePath, 'utf8');
function slice(start, end) {
  const a = source.indexOf(start), b = source.indexOf(end, a);
  assert(a >= 0 && b > a, `Editor helper boundary missing: ${start}`);
  return source.slice(a, b);
}
const helpers = slice('let _={Action:', 'function L(e)') +
  slice('function Z(', 'function Y(') + slice('function Y(', 'f.yX.Top') +
  slice('function te(e,t)', 'function tt(');
const inspect = new Function('xml', 'models', helpers + `
  const H={VERTICAL:'vertical',HORIZONTAL:'horizontal'},B=[],M=new Map();let nextId=0;
  function G(){return 'g_'+(++nextId);}
  const parsed=D(xml), templates=D(models).templates, graph=Z(parsed,H.VERTICAL,templates);
  const roundtrip=Y(graph.nodes,graph.edges);
  const serialize=(tree)=>{const lines=['<root BTCPP_format="4">','  <BehaviorTree ID="HuashuEditable">'];I(tree,lines,2);lines.push('  </BehaviorTree>','</root>');return lines.join('\\n')+'\\n';};
  const roundtripXml=serialize(roundtrip.mainTree);
  const sequence=graph.nodes.find(n=>n.data.name==='Sequence'&&graph.edges.some(e=>e.source===n.id&&graph.nodes.find(c=>c.id===e.target)?.data.name==='HrcsiSetDO'));
  const childIndices=graph.edges.map((e,i)=>e.source===sequence.id?i:-1).filter(i=>i>=0);
  const edges=graph.edges.slice();[edges[childIndices[0]],edges[childIndices[1]]]=[edges[childIndices[1]],edges[childIndices[0]]];
  const motion=graph.nodes.find(n=>n.data.name==='RobotMoveJ');motion.data.ports.target='OFFLINE_EDIT_POINT';motion.data.ports.speed='v10';
  const edited=Y(graph.nodes,te(edges,sequence.id));
  return {parsed,roundtrip,roundtripXml,edited,editedXml:serialize(edited.mainTree),
    nodes:graph.nodes.map(n=>({name:n.data.name,category:n.data.category})),templates};
`);
const result = inspect(fs.readFileSync(treePath, 'utf8'), fs.readFileSync(modelsPath, 'utf8'));
function canonical(node) {
  return {name:node.name,ports:node.ports,children:(node.children||[]).map(canonical)};
}
assert.deepEqual(canonical(result.parsed.mainTree),canonical(result.roundtrip.mainTree),'Save changes node order or parameters');
assert.equal(result.nodes[0].name,'RobotTask');assert.equal(result.nodes[0].category,2);
assert(result.nodes.filter(n=>n.name==='RobotSync').every(n=>n.category===0));
const task = result.templates.find(n=>n.name==='RobotTask');
assert(task.ports.some(p=>p.name==='request_timeout_ms'&&String(p.default)==='100000000'));
assert(!task.ports.some(p=>p.name==='profile'));
assert(result.templates.find(n=>n.name==='RobotMoveJ').ports.some(p=>p.name==='speed'&&p.default==='v5'));
const firstTwo=result.edited.mainTree.children[0].children[0].children.slice(0,2);
assert.equal(firstTwo[0].ports.do_num,'2');assert.equal(firstTwo[1].ports.do_num,'1');
function find(node,name){if(node.name===name)return node;for(const child of node.children||[]){const result=find(child,name);if(result)return result;}}
const changed=find(result.edited.mainTree,'RobotMoveJ');
assert.equal(changed.ports.target,'OFFLINE_EDIT_POINT');assert.equal(changed.ports.speed,'v10');
fs.mkdirSync(outputDir,{recursive:true});
fs.writeFileSync(path.join(outputDir,'editor-roundtrip.xml'),result.roundtripXml);
fs.writeFileSync(path.join(outputDir,'editor-edit-test.xml'),result.editedXml);
const report={editor_sha256:crypto.createHash('sha256').update(source).digest('hex'),
  nodes:result.nodes.length,sync_nodes:result.nodes.filter(n=>n.name==='RobotSync').length,
  roundtrip_preserves_tree:true,reorder_and_parameter_edits:true,new_ports_and_categories:true,robot_commands_sent:0};
fs.writeFileSync(path.join(outputDir,'editor-tests.json'),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));
