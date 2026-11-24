// Dependency-free renderer for the small, intentionally simple Markdown guide.
import { readFileSync, mkdirSync, writeFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { join } from "node:path";
const root = fileURLToPath(new URL(".", import.meta.url));
const output = join(root, "site");
mkdirSync(output, { recursive: true });
const chapters = [
  ["introduction","Introduction"],["getting-started","Getting started"],["coordinates","Coordinates & units"],
  ["localization","Tracking & localization"],["motion","Motion commands"],["routes","Competition routes"],
  ["programmed","Programmed autonomous"],["mechanisms","Mechanisms"],["calibration","Calibration"],
  ["recording","Logging & profiling"],["api","API map"],["development","Development"]
];
const esc = s => s.replaceAll("&","&amp;").replaceAll("<","&lt;").replaceAll(">","&gt;").replaceAll('"',"&quot;");
const inline = s => esc(s).replace(/\[([^\]]+)\]\((https:\/\/[^\s)]+|[a-z-]+\.html)\)/g,'<a href="$2">$1</a>').replace(/`([^`]+)`/g,"<code>$1</code>");
function markdown(source) {
  let html = "", para = [], list = false, code = null;
  const flush = () => { if (para.length) { html += "<p>"+inline(para.join(" "))+"</p>"; para=[]; } };
  const closeList = () => { if(list){html+="</ul>";list=false;} };
  for(const line of source.split("\n")){
    if(code!==null){if(line.startsWith("```")){html+="<pre><code>"+esc(code.join("\n"))+"</code></pre>";code=null;}else code.push(line);continue;}
    if(line.startsWith("```")){flush();closeList();code=[];continue;}
    const heading=line.match(/^(#{1,3}) (.+)$/);
    if(heading){flush();closeList();const n=heading[1].length;const id=heading[2].toLowerCase().replace(/[^a-z0-9]+/g,"-");html+="<h"+n+' id="'+id+'">'+inline(heading[2])+"</h"+n+">";continue;}
    if(line.startsWith("- ")){flush();if(!list){html+="<ul>";list=true;}html+="<li>"+inline(line.slice(2))+"</li>";continue;}
    if(!line.trim()){flush();closeList();continue;}
    closeList();para.push(line);
  }
  flush();closeList();if(code!==null)throw Error("Unclosed code block");
  return html;
}
const style=`*{box-sizing:border-box}body{margin:0;font:16px/1.72 system-ui,sans-serif;color:#202938;background:#fff}header{border-bottom:1px solid #dce2eb;padding:14px 24px;display:flex;align-items:center;justify-content:space-between;gap:16px}header strong{font-size:22px;letter-spacing:-.04em}header a{color:inherit;text-decoration:none}header small{color:#64748b}main{max-width:1240px;margin:auto;display:grid;grid-template-columns:240px minmax(0,780px);gap:56px;padding:30px 24px 80px}nav{border-right:1px solid #dce2eb;padding-right:20px;position:sticky;top:24px;align-self:start}nav a{display:block;font-size:14px;text-decoration:none;color:#475569;padding:5px 10px;border-radius:5px}nav a[aria-current]{background:#e9f2ff;color:#154b98;font-weight:600}nav h2{font-size:12px;text-transform:uppercase;letter-spacing:.08em;color:#64748b}article h1{font-size:38px;line-height:1.18;letter-spacing:-.045em;margin:12px 0 24px}article h2{font-size:23px;line-height:1.3;margin-top:36px;letter-spacing:-.025em}article p,article li{max-width:75ch}a{color:#185bb5;text-underline-offset:3px}pre{background:#f4f7fb;border:1px solid #e2e8f0;border-radius:9px;padding:17px;overflow:auto;font-size:13px;line-height:1.6}code{font-family:ui-monospace,SFMono-Regular,Consolas,monospace}p code,li code{background:#eef2f8;padding:2px 4px;border-radius:3px}footer{margin-top:48px;border-top:1px solid #dce2eb;padding-top:20px;display:flex;justify-content:space-between;gap:15px;font-size:14px}button{background:transparent;border:1px solid #aab6c4;border-radius:5px;padding:6px 10px;color:inherit;cursor:pointer}#search{width:100%;padding:8px;border:1px solid #cbd5e1;border-radius:5px;margin-bottom:10px;background:inherit;color:inherit}@media(max-width:760px){main{display:block;padding:20px}nav{position:static;border-right:0;border-bottom:1px solid #dce2eb;padding:0 0 20px;margin-bottom:24px}nav a{display:inline-block}article h1{font-size:31px}header small{display:none}}@media(prefers-color-scheme:dark){body{background:#111827;color:#e2e8f0}header,nav,footer{border-color:#334155}nav a{color:#cbd5e1}nav a[aria-current]{background:#233752;color:#bfdbfe}a{color:#93c5fd}pre,p code,li code{background:#1e293b;border-color:#334155}}`;
for(let i=0;i<chapters.length;i++){
  const [slug,title]=chapters[i];const body=markdown(readFileSync(join(root,"src",slug+".md"),"utf8"));
  const nav=chapters.map(([s,t])=>'<a href="'+s+'.html"'+(s===slug?' aria-current="page"':'')+'>'+esc(t)+'</a>').join("");
  const previous=i?'<a href="'+chapters[i-1][0]+'.html">← '+esc(chapters[i-1][1])+'</a>':"<span></span>";
  const next=i+1<chapters.length?'<a href="'+chapters[i+1][0]+'.html">'+esc(chapters[i+1][1])+' →</a>':"";
  const page='<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+esc(title)+' · Ozton</title><meta name="description" content="Practical Rust robotics documentation for VEX V5"><style>'+style+'</style></head><body><header><a href="index.html"><strong>Ozton</strong> <small>Documentation · development</small></a><a href="https://github.com/gmhs-robotics/ozton">GitHub</a></header><main><nav aria-label="Chapters"><h2>Guide</h2><input id="search" placeholder="Find a chapter" aria-label="Filter chapters">'+nav+'</nav><article>'+body+'<footer>'+previous+next+'</footer></article></main><script>document.getElementById("search").addEventListener("input",e=>document.querySelectorAll("nav a").forEach(a=>a.hidden=!a.textContent.toLowerCase().includes(e.target.value.toLowerCase())));</script></body></html>';
  writeFileSync(join(output,slug+".html"),page);
  if(i===0)writeFileSync(join(output,"index.html"),page);
}
