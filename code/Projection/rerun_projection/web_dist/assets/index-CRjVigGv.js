var Z=t=>{throw TypeError(t)};var K=(t,e,i)=>e.has(t)||Z("Cannot "+i);var n=(t,e,i)=>(K(t,e,"read from private field"),i?i.call(t):e.get(t)),m=(t,e,i)=>e.has(t)?Z("Cannot add the same private member more than once"):e instanceof WeakSet?e.add(t):e.set(t,i),f=(t,e,i,r)=>(K(t,e,"write to private field"),r?r.call(t,i):e.set(t,i),i),p=(t,e,i)=>(K(t,e,"access private method"),i);(function(){const e=document.createElement("link").relList;if(e&&e.supports&&e.supports("modulepreload"))return;for(const s of document.querySelectorAll('link[rel="modulepreload"]'))r(s);new MutationObserver(s=>{for(const a of s)if(a.type==="childList")for(const c of a.addedNodes)c.tagName==="LINK"&&c.rel==="modulepreload"&&r(c)}).observe(document,{childList:!0,subtree:!0});function i(s){const a={};return s.integrity&&(a.integrity=s.integrity),s.referrerPolicy&&(a.referrerPolicy=s.referrerPolicy),s.crossOrigin==="use-credentials"?a.credentials="include":s.crossOrigin==="anonymous"?a.credentials="omit":a.credentials="same-origin",a}function r(s){if(s.ep)return;s.ep=!0;const a=i(s);fetch(s.href,a)}})();async function se(){return(await fetch("/api/bootstrap")).json()}async function he(t){return(await fetch("/api/config/source",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(t)})).json()}async function pe(t){return(await fetch("/api/config/projection",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(t)})).json()}async function fe(t){return(await fetch("/api/select/2d",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(t)})).json()}async function me(t){return(await fetch("/api/select/3d",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(t)})).json()}async function _e(){return(await fetch("/api/pairs/lock",{method:"POST"})).json()}async function ge(){return(await fetch("/api/pairs/delete-last",{method:"POST"})).json()}async function we(){return(await fetch("/api/pairs/clear",{method:"POST"})).json()}async function ye(){return(await fetch("/api/frame/next",{method:"POST"})).json()}async function ve(){return(await fetch("/api/frame/prev",{method:"POST"})).json()}async function oe(t){return(await fetch("/api/frame/set",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({frame_index:t})})).json()}const ae=Object.freeze(Object.defineProperty({__proto__:null,applyProjection:pe,applySource:he,clearPairs:we,deleteLastPair:ge,fetchBootstrap:se,lockPair:_e,nextFrame:ye,prevFrame:ve,select2d:fe,select3d:me,setFrame:oe},Symbol.toStringTag,{value:"Module"}));function ee(t){return t.map(e=>e.join(", ")).join(`
`)}function te(t,e){try{const i=t.split(`
`).map(r=>r.trim()).filter(Boolean).map(r=>r.split(",").map(s=>Number(s.trim())));return i.length>0?i:e}catch{return e}}function be(t){return t.lockedPairs.length===0?'<div class="empty-state">No locked pairs yet.</div>':t.lockedPairs.map(e=>`<div class="pair-row"><span class="pair-chip" style="--chip:${`rgb(${e.color_rgb.join(",")})`}">P${e.pair_id}</span><span>${e.projected_pixel.map(r=>r.toFixed(1)).join(", ")}</span></div>`).join("")}function Se(t,e,i,r){const s=e.startup.state==="building",a=e.currentSelection!==null,c=s?'<div class="status-banner status-banner-loading">Preparing recording. The page is up now, and the full bag is still being indexed in the background.</div>':e.startup.state==="error"?`<div class="status-banner status-banner-error">Recording failed to initialize: ${e.startup.error??"Unknown error"}</div>`:"";t.innerHTML=`
    <div class="sidebar-shell">
      <section class="sidebar-hero">
        <div class="hero-kicker">Projection Workbench</div>
        <h1>Parameters</h1>
        <p>Use the Rerun time bar under the viewer as the single playback control. The right sidebar only handles bag source and calibration edits.</p>
        ${c}
      </section>

      <section class="sidebar-section">
        <div class="section-heading">
          <div>
            <div class="section-kicker">Viewer Timeline</div>
            <h2>Timeline</h2>
          </div>
        </div>
        <div class="empty-state">Use the built-in Rerun transport controls below the viewer to scrub, play, and pause the bag timeline.</div>
      </section>

      <section class="sidebar-section">
        <div class="section-heading">
          <div>
            <div class="section-kicker">Data Source</div>
            <h2>ROS Inputs</h2>
          </div>
        </div>
        <label>Bag Path<input id="bag_file" value="${e.draftSource.bag_file}" /></label>
        <label>YAML Path<input id="yaml_path" value="${e.draftSource.yaml_path}" /></label>
        <label>Semantic Image Topic<input id="image_topic" value="${e.draftSource.image_topic}" /></label>
        <label>Detection Overlay Topic<input id="overlay_image_topic" value="${e.draftSource.overlay_image_topic}" /></label>
        <label>Point Cloud Topic<input id="pointcloud_topic" value="${e.draftSource.pointcloud_topic}" /></label>
        <button id="applySource" class="button-primary" ${s?"disabled":""}>Apply Source</button>
      </section>

      <section class="sidebar-section">
        <div class="section-heading">
          <div>
            <div class="section-kicker">Projection Params</div>
            <h2>Calibration</h2>
          </div>
        </div>

        <div class="param-group">
          <h3>Camera Matrix</h3>
          <div class="compact-grid">
            <label>Image Width<input id="image_width" type="number" value="${e.draftProjection.image_width}" /></label>
            <label>Image Height<input id="image_height" type="number" value="${e.draftProjection.image_height}" /></label>
          </div>
          <label>Intrinsic Matrix<textarea id="camera_matrix">${ee(e.draftProjection.camera_matrix)}</textarea></label>
        </div>

        <div class="param-group">
          <h3>Distortion</h3>
          <div class="compact-grid">
            <label>Min Depth<input id="min_depth" type="number" step="0.01" value="${e.draftProjection.min_depth}" /></label>
            <label>Coefficients<input id="distortion_coeffs" value="${e.draftProjection.distortion_coeffs.join(", ")}" /></label>
          </div>
        </div>

        <div class="param-group">
          <h3>LiDAR To Camera</h3>
          <label>Extrinsic Matrix<textarea id="lidar_to_camera">${ee(e.draftProjection.lidar_to_camera)}</textarea></label>
        </div>

        <button id="applyProjection" class="button-primary" ${s?"disabled":""}>Apply Projection</button>
      </section>

      <section class="sidebar-section">
        <div class="section-heading">
          <div>
            <div class="section-kicker">Locked Pairs</div>
            <h2>Comparisons</h2>
          </div>
          <div class="section-meta">${e.lockedPairs.length} saved</div>
        </div>
        <div class="button-row">
          <button id="lockPair" class="button-primary" ${a&&!s?"":"disabled"}>Lock Pair</button>
          <button id="deleteLastPair" class="button-secondary" ${s?"disabled":""}>Delete Last</button>
          <button id="clearPairs" class="button-danger" ${s?"disabled":""}>Clear All</button>
        </div>
        <div class="pairs-list">${be(e)}</div>
      </section>
    </div>
  `;const l=()=>({bag_file:t.querySelector("#bag_file").value,yaml_path:t.querySelector("#yaml_path").value,image_topic:t.querySelector("#image_topic").value,overlay_image_topic:t.querySelector("#overlay_image_topic").value,pointcloud_topic:t.querySelector("#pointcloud_topic").value}),d=()=>({image_width:Number(t.querySelector("#image_width").value),image_height:Number(t.querySelector("#image_height").value),min_depth:Number(t.querySelector("#min_depth").value),camera_matrix:te(t.querySelector("#camera_matrix").value,e.draftProjection.camera_matrix),distortion_coeffs:t.querySelector("#distortion_coeffs").value.split(",").map(h=>Number(h.trim())),lidar_to_camera:te(t.querySelector("#lidar_to_camera").value,e.draftProjection.lidar_to_camera)});t.querySelector("#applySource").onclick=async()=>{const h=await i.applySource(l());r(h)},t.querySelector("#applyProjection").onclick=async()=>{const h=await i.applyProjection(d());r(h)},t.querySelector("#lockPair").onclick=async()=>{const h=await i.lockPair();r(h)},t.querySelector("#deleteLastPair").onclick=async()=>{const h=await i.deleteLastPair();r(h)},t.querySelector("#clearPairs").onclick=async()=>{const h=await i.clearPairs();r(h)}}function Pe(){return{draftSource:{bag_file:"",yaml_path:"",image_topic:"",overlay_image_topic:"",pointcloud_topic:""},draftProjection:{image_width:0,image_height:0,camera_matrix:[],distortion_coeffs:[],lidar_to_camera:[],min_depth:.05},currentFrame:null,currentSelection:null,lockedPairs:[],rerunGrpcUrl:"",startup:{state:"idle",error:null}}}function xe(t,e){t.draftSource={bag_file:e.config.bag_file,yaml_path:e.config.yaml_path??"",image_topic:e.config.image_topic,overlay_image_topic:e.config.overlay_image_topic,pointcloud_topic:e.config.pointcloud_topic},t.draftProjection={image_width:e.config.image_width,image_height:e.config.image_height,camera_matrix:e.config.camera_matrix,distortion_coeffs:e.config.distortion_coeffs,lidar_to_camera:e.config.lidar_to_camera,min_depth:e.config.min_depth},t.currentFrame=e.current_frame??null,t.currentSelection=e.current_selection??null,t.lockedPairs=e.locked_pairs??[],t.rerunGrpcUrl=e.rerun_grpc_url??"",t.startup=e.startup??{state:"ready",error:null}}const je="modulepreload",$e=function(t){return"/"+t},ie={},ke=function(e,i,r){let s=Promise.resolve();if(i&&i.length>0){document.getElementsByTagName("link");const c=document.querySelector("meta[property=csp-nonce]"),l=(c==null?void 0:c.nonce)||(c==null?void 0:c.getAttribute("nonce"));s=Promise.allSettled(i.map(d=>{if(d=$e(d),d in ie)return;ie[d]=!0;const h=d.endsWith(".css"),L=h?'[rel="stylesheet"]':"";if(document.querySelector(`link[href="${d}"]${L}`))return;const y=document.createElement("link");if(y.rel=h?"stylesheet":je,h||(y.as="script"),y.crossOrigin="",y.href=d,l&&y.setAttribute("nonce",l),document.head.appendChild(y),h)return new Promise((F,V)=>{y.addEventListener("load",F),y.addEventListener("error",()=>V(new Error(`Unable to preload CSS for ${d}`)))})}))}function a(c){const l=new Event("vite:preloadError",{cancelable:!0});if(l.payload=c,window.dispatchEvent(l),!l.defaultPrevented)throw c}return s.then(c=>{for(const l of c||[])l.status==="rejected"&&a(l.reason);return e().catch(a)})};let Y=null,Q=null;async function Ee(t){return(await ke(async()=>{const{default:e}=await import("./re_viewer-D5vaVejy.js");return{default:e}},[])).default}async function Te(t,e){//!<INLINE-MARKER-OPEN>
const i=t?new URL("./re_viewer_bg.wasm",t):new URL("/assets/re_viewer_bg-BrfzqDf3.wasm",import.meta.url),r=await fetch(i);if(!r.ok)throw new Error(`Failed to fetch viewer WASM: ${r.status} ${r.statusText}`);return Fe(r,e);//!<INLINE-MARKER-CLOSE>
}function Le(t){const e=t.headers.get("rerun-final-length");if(e!=null)return parseInt(e,10);if(t.headers.get("content-encoding")==="gzip"){const r=t.headers.get("x-goog-meta-uncompressed-size");if(r!=null)return parseInt(r,10);const s=t.headers.get("content-length");if(s!=null)return parseInt(s,10)*3}const i=t.headers.get("content-length");return i!=null?parseInt(i,10):null}function Fe(t,e){const i=Le(t);if(!t.body)return t;let r=0;const s=t.body,a=new ReadableStream({async start(c){const l=s.getReader();for(;;){const{done:d,value:h}=await l.read();if(d)break;r+=h.byteLength,e==null||e(r,i),c.enqueue(h)}c.close()}});return new Response(a,{status:t.status,statusText:t.statusText,headers:t.headers})}function qe(t){return(t/(1024*1024)).toFixed(1)+" MiB"}async function Oe(t,e){(!Y||!Q)&&([Y,Q]=await Promise.all([Ee(),WebAssembly.compileStreaming(Te(t,e))]));let i=Y();return await i({module_or_path:Q}),class extends i.WebHandle{free(){super.free(),i.deinit()}}}let k=null;function Ce(){const t=new Uint8Array(16);return crypto.getRandomValues(t),Array.from(t).map(e=>e.toString(16).padStart(2,"0")).join("")}function Ie(t){return new Promise(e=>setTimeout(e,t))}function re(t){return new URL(t,window.location.href).toString()}var X,o,w,v,S,P,E,I,u,ce,j,U,b,$,O,H;class Ae{constructor(){m(this,u);m(this,X,Ce());m(this,o,null);m(this,w,null);m(this,v,null);m(this,S,"stopped");m(this,P,!1);m(this,E,!1);m(this,I,new Set);m(this,j,new Map);m(this,O,()=>{});m(this,H,()=>{k==null||k();const e=n(this,w),i=e.getBoundingClientRect(),r=()=>{e.style.left=i.left+"px",e.style.top=i.top+"px",e.style.width=i.width+"px",e.style.height=i.height+"px"},s=()=>e.removeAttribute("style"),a=c=>setTimeout(()=>requestAnimationFrame(c),le);e.classList.add(_.fullscreen_base,_.fullscreen_rect),r(),requestAnimationFrame(()=>{n(this,P)&&(e.classList.add(_.transition),a(()=>{n(this,P)&&(s(),document.body.classList.add(_.hide_scrollbars),document.documentElement.classList.add(_.hide_scrollbars),p(this,u,U).call(this,"fullscreen",!0))}))}),f(this,O,()=>{document.body.classList.remove(_.hide_scrollbars),document.documentElement.classList.remove(_.hide_scrollbars),r(),e.classList.remove(_.fullscreen_rect),a(()=>{n(this,P)||(s(),e.classList.remove(_.fullscreen_base,_.transition))}),k=null,f(this,P,!1),p(this,u,U).call(this,"fullscreen",!1)}),k=()=>n(this,O).call(this),f(this,P,!0)});Ne(),Re()}async start(e,i,r){if(i??(i=document.body),r??(r={}),r=r&&{...r},f(this,E,r.allow_fullscreen||!1),n(this,S)!=="stopped")return;f(this,S,"starting"),p(this,u,$).call(this),f(this,w,document.createElement("canvas")),n(this,w).style.width=r.width??"640px",n(this,w).style.height=r.height??"360px",i.append(n(this,w)),f(this,v,document.createElement("div")),n(this,v).innerHTML=`
      <div style="display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100%; background-color: #1c1c1c; font-family: sans-serif; color: white;">
        <div style="margin-bottom: 16px;">Loading Rerun…</div>
        <div style="width: 200px;">
          <div style="background: #333; border-radius: 4px; height: 6px; overflow: hidden;">
            <div class="rerun-progress-bar" style="background: white; height: 100%; width: 0%; transition: width 0.2s;"></div>
          </div>
          <div class="rerun-progress-text" style="margin-top: 6px; font-size: 12px; color: #999;"></div>
        </div>
      </div>
    `,n(this,v).style.position="absolute",n(this,v).style.inset="0",i.style.position="relative",i.append(n(this,v));const s=n(this,v).querySelector(".rerun-progress-bar"),a=n(this,v).querySelector(".rerun-progress-text"),c=(g,x)=>{if(x!=null&&x>0){const A=Math.min(g/x*100,100);s.style.width=A.toFixed(1)+"%",a.textContent=`${Math.round(A)}%`}else a.textContent=qe(g)};await Ie(0);let l=r==null?void 0:r.base_url;l&&delete r.base_url;let d;try{d=await Oe(l,c)}catch(g){throw p(this,u,$).call(this),p(this,u,b).call(this,"Failed to load rerun",String(g)),g}if(n(this,S)!=="starting"){p(this,u,$).call(this);return}const h=n(this,E)?{get_state:()=>n(this,P),on_toggle:()=>this.toggle_fullscreen()}:void 0,L=g=>{p(this,u,ce).call(this,g);let x=JSON.parse(g);p(this,u,U).call(this,x.type,x)},y=r.login?{signed_in_url:re(r.login.signed_in_url),signed_out_url:re(r.login.signed_out_url)}:void 0;f(this,o,new d({...r,login:y,fullscreen:h,on_viewer_event:L}));try{await n(this,o).start(n(this,w))}catch(g){throw p(this,u,$).call(this),p(this,u,b).call(this,"Failed to start",String(g)),g}if(n(this,S)!=="starting"){p(this,u,$).call(this);return}p(this,u,$).call(this),f(this,S,"ready"),p(this,u,U).call(this,"ready"),e&&this.open(e);let F=this;function V(){var g,x,A;(g=n(F,o))!=null&&g.has_panicked()?p(A=F,u,b).call(A,"Rerun has crashed.",(x=n(F,o))==null?void 0:x.panic_message()):setTimeout(V,1e3)}V()}_on_raw_event(e){return n(this,I).add(e),()=>n(this,I).delete(e)}on(e,i){const r=n(this,j).get(e)??new Map;return r.set(i,{once:!1}),n(this,j).set(e,r),()=>r.delete(i)}once(e,i){const r=n(this,j).get(e)??new Map;return r.set(i,{once:!0}),n(this,j).set(e,r),()=>r.delete(i)}off(e,i){const r=n(this,j).get(e);r?r.delete(i):console.warn("Attempted to call `WebViewer.off` with an unregistered callback. Are you passing in the same function instance?")}get canvas(){return n(this,w)}get ready(){return n(this,S)==="ready"}open(e,i={}){if(!n(this,o))throw new Error(`attempted to open \`${e}\` in a stopped viewer`);const r=Array.isArray(e)?e:[e];for(const s of r)try{n(this,o).add_receiver(s,i.follow_if_http)}catch(a){throw p(this,u,b).call(this,"Failed to open recording",String(a)),a}}close(e){if(!n(this,o))throw new Error(`attempted to close \`${e}\` in a stopped viewer`);const i=Array.isArray(e)?e:[e];for(const r of i)try{n(this,o).remove_receiver(r)}catch(s){throw p(this,u,b).call(this,"Failed to close recording",String(s)),s}}stop(){var e,i,r;if(n(this,S)!=="stopped"){n(this,E)&&n(this,w)&&n(this,P)&&n(this,O).call(this),f(this,S,"stopped"),(e=n(this,w))==null||e.remove(),p(this,u,$).call(this);try{(i=n(this,o))==null||i.destroy(),(r=n(this,o))==null||r.free()}catch(s){throw f(this,o,null),s}f(this,w,null),f(this,o,null),f(this,v,null),f(this,P,!1),f(this,E,!1)}}open_channel(e="rerun-io/web-viewer"){if(!n(this,o))throw new Error(`attempted to open channel "${e}" in a stopped web viewer`);const i=crypto.randomUUID();try{n(this,o).open_channel(i,e)}catch(l){throw p(this,u,b).call(this,"Failed to open channel",String(l)),l}const r=l=>{if(!n(this,o))throw new Error(`attempted to send data through channel "${e}" to a stopped web viewer`);try{n(this,o).send_rrd_to_channel(i,l)}catch(d){throw p(this,u,b).call(this,"Failed to send data",String(d)),d}},s=l=>{if(!n(this,o))throw new Error(`attempted to send data through channel "${e}" to a stopped web viewer`);try{n(this,o).send_table_to_channel(i,l)}catch(d){throw p(this,u,b).call(this,"Failed to send table",String(d)),d}},a=()=>{if(!n(this,o))throw new Error(`attempted to send data through channel "${e}" to a stopped web viewer`);try{n(this,o).close_channel(i)}catch(l){throw p(this,u,b).call(this,"Failed to close channel",String(l)),l}},c=()=>n(this,S);return new Ue(r,s,a,c)}override_panel_state(e,i){if(!n(this,o))throw new Error(`attempted to set ${e} panel to ${i} in a stopped web viewer`);try{n(this,o).override_panel_state(e,i)}catch(r){throw p(this,u,b).call(this,"Failed to override panel state",String(r)),r}}toggle_panel_overrides(e){if(!n(this,o))throw new Error("attempted to toggle panel overrides in a stopped web viewer");try{n(this,o).toggle_panel_overrides(e)}catch(i){throw p(this,u,b).call(this,"Failed to toggle panel overrides",String(i)),i}}get_active_recording_id(){if(!n(this,o))throw new Error("attempted to get active recording id in a stopped web viewer");return n(this,o).get_active_recording_id()??null}set_active_recording_id(e){if(!n(this,o))throw new Error(`attempted to set active recording id to ${e} in a stopped web viewer`);n(this,o).set_active_recording_id(e)}get_playing(e){if(!n(this,o))throw new Error("attempted to get play state in a stopped web viewer");return n(this,o).get_playing(e)||!1}set_playing(e,i){if(!n(this,o))throw new Error(`attempted to set play state to ${i?"playing":"paused"} in a stopped web viewer`);n(this,o).set_playing(e,i)}get_current_time(e,i){if(!n(this,o))throw new Error("attempted to get current time in a stopped web viewer");return n(this,o).get_time_for_timeline(e,i)||0}set_current_time(e,i,r){if(!n(this,o))throw new Error(`attempted to set current time to ${r} in a stopped web viewer`);n(this,o).set_time_for_timeline(e,i,r)}get_active_timeline(e){if(!n(this,o))throw new Error("attempted to get active timeline in a stopped web viewer");return n(this,o).get_active_timeline(e)??null}set_active_timeline(e,i){if(!n(this,o))throw new Error(`attempted to set active timeline to ${i} in a stopped web viewer`);n(this,o).set_active_timeline(e,i)}get_time_range(e,i){if(!n(this,o))throw new Error("attempted to get time range in a stopped web viewer");return n(this,o).get_timeline_time_range(e,i)}toggle_fullscreen(){if(n(this,E)){if(!n(this,o)||!n(this,w))throw new Error("attempted to toggle fullscreen mode in a stopped web viewer");n(this,P)?n(this,O).call(this):n(this,H).call(this)}}set_credentials(e,i){if(!n(this,o))throw new Error("attempted to set credentials in a stopped web viewer");n(this,o).set_credentials(e,i)}}X=new WeakMap,o=new WeakMap,w=new WeakMap,v=new WeakMap,S=new WeakMap,P=new WeakMap,E=new WeakMap,I=new WeakMap,u=new WeakSet,ce=function(e){for(const i of n(this,I))i(e)},j=new WeakMap,U=function(e,...i){setTimeout(()=>{const r=n(this,j).get(e);if(r)for(const[s,{once:a}]of[...r.entries()])s(...i),a&&r.delete(s)},0)},b=function(e,i){var r;if(console.error("WebViewer failure:",e,i),(r=this.canvas)!=null&&r.parentElement){const s=this.canvas.parentElement;s.innerHTML=`
        <div style="display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100%; color: white; font-family: sans-serif; background-color: #1c1c1c;">
          <h1 class="rerun-fail-message"></h1>
          <pre class="rerun-fail-error" style="text-align: left; white-space: pre-wrap; word-break: break-word; max-width: 90vw;"></pre>
          <button class="rerun-fail-clear-cache">Clear caches and reload</button>
        </div>
      `,s.querySelector(".rerun-fail-message").textContent=e;const a=s.querySelector(".rerun-fail-error");i?a.textContent=i:a.remove(),s.querySelector(".rerun-fail-clear-cache").addEventListener("click",async()=>{if("caches"in window){const c=await caches.keys();await Promise.all(c.map(l=>caches.delete(l)))}window.location.reload()})}this.stop()},$=function(){var e;(e=n(this,v))==null||e.remove(),f(this,v,null)},O=new WeakMap,H=new WeakMap;var N,R,D,B,W;class Ue{constructor(e,i,r,s){m(this,N);m(this,R);m(this,D);m(this,B);m(this,W,!1);f(this,N,e),f(this,R,i),f(this,D,r),f(this,B,s)}get ready(){return!n(this,W)&&n(this,B).call(this)==="ready"}send_rrd(e){this.ready&&n(this,N).call(this,e)}send_table(e){this.ready&&n(this,R).call(this,e)}close(){this.ready&&(n(this,D).call(this),f(this,W,!0))}}N=new WeakMap,R=new WeakMap,D=new WeakMap,B=new WeakMap,W=new WeakMap;const _={hide_scrollbars:"rerun-viewer-hide-scrollbars",fullscreen_base:"rerun-viewer-fullscreen-base",fullscreen_rect:"rerun-viewer-fullscreen-rect",transition:"rerun-viewer-transition"},le=100,Me=`
  html.${_.hide_scrollbars},
  body.${_.hide_scrollbars} {
    scrollbar-gutter: auto !important;
    overflow: hidden !important;
  }

  .${_.fullscreen_base} {
    position: fixed;
    z-index: 99999;
  }

  .${_.transition} {
    transition: all ${le/1e3}s linear;
  }

  .${_.fullscreen_rect} {
    left: 0;
    top: 0;
    width: 100%;
    height: 100%;
  }
`;function Ne(){const t="__rerun_viewer_style";if(document.getElementById(t))return;const e=document.createElement("style");e.id=t,e.appendChild(document.createTextNode(Me)),document.head.appendChild(e)}function Re(){window.addEventListener("keyup",t=>{t.code==="Escape"&&(k==null||k())})}function De(t){var s;const e=new Set(["world/ego_vehicle/lidar","world/ego_vehicle/semantic_camera/projected_points","world/ego_vehicle/overlay_camera/projected_points"]);if(t.type!=="entity"||!t.entity_path||!e.has(t.entity_path))return null;const i=t.instance_id??"none",r=((s=t.position)==null?void 0:s.join(","))??"none";return`${t.entity_path}:${i}:${r}`}async function Be(t,e,i,r,s){const a=new Ae;await a.start(e,t,{hide_welcome_screen:!0,width:"100%",height:"100%"});let c=null,l=null;return a.on("recording_open",d=>{l=d.recording_id,a.set_active_timeline(d.recording_id,"frame"),a.set_current_time(d.recording_id,"frame",s.initialFrameIndex)}),a.on("time_update",d=>{s.onTimeUpdate(Math.max(0,Math.round(d.time)))}),a.on("pause",async()=>{await s.onPause(s.getFrameIndex())}),a.on("selection_change",async d=>{const h=d.items[0];if(!h)return;const L=De(h);if(L===null||L===c)return;c=L;const y=s.getFrameIndex();await s.ensureFrameSynced(y);const F=await r.select3d({frame_index:y,entity_path:h.entity_path,instance_id:h.instance_id??null,position:h.position??null});i(F)}),{stop:()=>{if(l!==null)try{a.close(e)}catch{}a.stop()}}}const M=Pe();let T=null,z="",q=0,G=0,C=null;async function ne(t){if(t===G)return null;const e=await oe(t);return G=t,e}async function We(t,e){T==null||T.stop(),T=await Be(document.querySelector("#rerunViewer"),t,de,ae,{initialFrameIndex:e,getFrameIndex:()=>q,onTimeUpdate:i=>{q=i},onPause:async i=>{q=i;const r=await ne(i);r&&await J(r)},ensureFrameSynced:async i=>{q=i;const r=await ne(i);r&&await J(r)}})}function Ve(){C===null&&(C=window.setTimeout(()=>{C=null,ue()},1e3))}function ze(){C!==null&&(window.clearTimeout(C),C=null)}function Ge(){Se(document.querySelector("#sidebar"),M,ae,de)}async function J(t){var r,s;const e=((r=t.current_frame)==null?void 0:r.frame_index)??q;xe(M,t),G=((s=M.currentFrame)==null?void 0:s.frame_index)??G,q=e;const i=M.rerunGrpcUrl;i&&i!==z?(z=i,await We(i,q)):!i&&z&&(T==null||T.stop(),T=null,z=""),M.startup.state==="building"?Ve():ze(),Ge()}function de(t){J(t)}async function ue(){const t=await se();await J(t)}ue();
