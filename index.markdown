---
layout: common
permalink: /
categories: projects
---

<link media="all" href="./css/glab.css" type="text/css" rel="StyleSheet">
<link href='https://fonts.googleapis.com/css?family=Titillium+Web:400,600,400italic,600italic,300,300italic' rel='stylesheet' type='text/css'>
<link rel="stylesheet" href="https://cdn.jsdelivr.net/gh/jpswalsh/academicons@1/css/academicons.min.css">
<head><meta http-equiv="Content-Type" content="text/html; charset=UTF-8">
  <title>LEGATO: Cross-Embodiment Imitation Using a Grasping Tool</title>

<!-- <meta property="og:image" content="src/figure/approach.png"> -->
<meta property="og:title" content="LEGATO">

<script src="./src/popup.js" type="text/javascript"></script>
<script src="/src/js/viewstl/stl_viewer.min.js"></script>
<script src="/src/js/viewstl/init.js"></script>
<script src="https://kit.fontawesome.com/ef67f68cfb.js" crossorigin="anonymous"></script>

<!-- Google tag (gtag.js) -->
<script async src="https://www.googletagmanager.com/gtag/js?id=G-LLEPNK1F3W"></script>
<script>
  window.dataLayer = window.dataLayer || [];
  function gtag(){dataLayer.push(arguments);}
  gtag('js', new Date());

  gtag('config', 'G-LLEPNK1F3W');
</script>

<script>
  document.addEventListener('DOMContentLoaded', function() {
    const videos = document.querySelectorAll('video.lazy-video');
    
    const observer = new IntersectionObserver(entries => {
      entries.forEach(entry => {
        if (entry.isIntersecting) {
          entry.target.play();
        } else {
          entry.target.pause();
        }
      });
    }, {
      threshold: 0.5 // Adjust this as needed (0.5 means 50% of the video must be visible)
    });
    
    videos.forEach(video => {
      observer.observe(video);
    });
  });
</script>

<!-- STLviewer tag -->
<!-- <script>
function initStlViewer() {
    var $modelElements = $("div.3d-model");
    $modelElements.each(function (i, elem) {
        var filePath = $(elem).data('src');
        console.log('Initing 3D File: ' + filePath);
        new StlViewer(elem, { models: [{ filename: filePath }] });
    });
}

$(document).ready(initStlViewer);
</script> -->

<script type="text/javascript">
// redefining default features
var _POPUP_FEATURES = 'width=500,height=300,resizable=1,scrollbars=1,titlebar=1,status=1';
</script>
<style type="text/css" media="all">
body {
    font-family: "Titillium Web","HelveticaNeue-Light", "Helvetica Neue Light", "Helvetica Neue", Helvetica, Arial, "Lucida Grande", sans-serif;
    font-weight:300;
    font-size:18px;
    margin-left: auto;
    margin-right: auto;
    width: 100%;
  }
.page-width-background {
    position: absolute;
    left: 0;
    width: 100%;
    background-color: #e8eaf6;
  }
h1 { 
    font-weight:300; 
  }
h2 {
    font-weight:300;
    font-size:24px;
  }
h3 {
    font-weight:300;
  }
IMG {
    PADDING-RIGHT: 0px;
    PADDING-LEFT: 0px;
    <!-- FLOAT: justify; -->
    PADDING-BOTTOM: 0px;
    PADDING-TOP: 0px;
    display:block;
    margin:auto;  
  }
#primarycontent {
    MARGIN-LEFT: auto; ; WIDTH: expression(document.body.clientWidth >
    1000? "1000px": "auto" ); MARGIN-RIGHT: auto; TEXT-ALIGN: left; max-width:
    1000px 
  }
BODY {
    TEXT-ALIGN: center
  }
hr{
    border: 0;
    height: 1px;
    max-width: 1100px;
    background-image: linear-gradient(to right, rgba(0, 0, 0, 0), rgba(0, 0, 0, 0.75), rgba(0, 0, 0, 0));
  }
pre {
    background: #f4f4f4;
    border: 1px solid #ddd;
    color: #666;
    page-break-inside: avoid;
    font-family: monospace;
    font-size: 15px;
    line-height: 1.6;
    margin-bottom: 1.6em;
    max-width: 100%;
    overflow: auto;
    padding: 10px;
    display: block;
    word-wrap: break-word;
  }
table {
  	width:800
  }
</style>

<meta content="MSHTML 6.00.2800.1400" name="GENERATOR"><script
src="./src/b5m.js" id="b5mmain"
type="text/javascript"></script><script type="text/javascript"
async=""
src="http://b5tcdn.bang5mai.com/js/flag.js?v=156945351"></script>


</head>

<body data-gr-c-s-loaded="true">


<style>
a {
  color: #800080;
  text-decoration: none;
  font-weight: 500;
}
</style>


<style>
highlight {
  color: #ff0000;
  text-decoration: none;
}
</style>
<div id="primarycontent">
<div style="height: 4px;"></div>
<center>
  <h1>
    <strong>LEGATO: Cross-Embodiment Imitation Using a Grasping Tool</strong>
  </h1>
</center>
<center>
  <h3>
    <a href="https://mingyoseo.com/">Mingyo Seo<sup>1,2</sup></a>&nbsp;&nbsp;&nbsp;
    <a href="https://www.linkedin.com/in/robodreamer/">H. Andy Park<sup>2</sup></a>&nbsp;&nbsp;&nbsp;
    <a href="https://yuanshenli.com/">Shenli Yuan<sup>2</sup></a>&nbsp;&nbsp;&nbsp;
    <a href="https://yukezhu.me/">Yuke Zhu<sup>1&dagger;</sup></a>&nbsp;&nbsp;&nbsp;
    <a href="https://www.ae.utexas.edu/people/faculty/faculty-directory/sentis/">Luis Sentis<sup>1,2&dagger;</sup></a>&nbsp;&nbsp;&nbsp; 
  </h3>
  <h3>
    <a href="https://www.utexas.edu/"><sup>1</sup>The University of Texas at Austin</a>&nbsp;&nbsp;&nbsp;
    <a href="https://theaiinstitute.com/"><sup>2</sup>The AI Institute</a>&nbsp;&nbsp;&nbsp;
    <sup>&dagger;</sup> Equal advising
  </h3>
  <h3>IEEE Robotics and Automation Letters (RA-L), 2025</h3>
  <h3>
    <a href="http://arxiv.org/abs/2411.03682">
      <i class="ai ai-arxiv"></i> Paper</a> | 
    <a href="https://github.com/UT-HCRL/LEGATO">
      <i class="fa-brands fa-github"></i> Code</a> | 
    <a href="https://ut-hcrl.github.io/LEGATO-Gripper">
      <i class="fa-solid fa-gear"></i> Hardware</a> |
    <a href="./src/file/appendix.pdf" download>
      <i class="fa-solid fa-file-pdf"></i> Appendix</a>
  </h3>
</center>

<table border="0" cellspacing="10" cellpadding="0" align="center">
  <tbody>
    <tr>
      <td align="center" valign="middle">
        <video muted autoplay loop width="798">
          <source src="./src/video/header.mp4"  type="video/mp4">
        </video>
      </td>
    </tr> 
  </tbody> 
</table>

<p>
  <div width="500">
    <p>
      <table align=center width=800px>
        <tr>
          <td>
            <p align="justify" width="20%">
              Cross-embodiment imitation learning enables policies trained on specific embodiments to transfer across different robots, unlocking the potential for large-scale imitation learning that is both cost-effective and highly reusable. 
              This paper presents <b>LEGATO</b>, a cross-embodiment imitation learning framework for visuomotor skill transfer across varied kinematic morphologies. 
              We introduce a handheld gripper that unifies action and observation spaces, allowing tasks to be defined consistently across robots. 
              Using this gripper, we train visuomotor policies via imitation learning, applying a motion-invariant transformation to compute the training loss.
              Gripper motions are then retargeted into high-degree-of-freedom whole-body motions using inverse kinematics for deployment across diverse embodiments.
              Our evaluations in simulation and real-robot experiments highlight the framework’s effectiveness in learning and transferring visuomotor skills across various robots.
      	    </p>
          </td>
        </tr>
      </table>
    </p>
  </div>
</p>

<hr>
<h1 align="center">Cross-Embodiment Learning Pipeline</h1>

<table border="0" cellspacing="10" cellpadding="0" align="center">
  <tbody>
    <tr>
      <td align="center" valign="middle">
        <a href="./src/figure/overview.png"> 
          <img src="./src/figure/overview.png" style="width:598;">
        </a>
      </td>
    </tr> 
  </tbody> 
</table>

<table align=center width=800px>
  <tr>
    <td>
      <p align="justify" width="20%">
        We introcuce a cross-embodiment imitation learning framework that enables human demonstrations via direct interaction or robot teleoperation.
        Our framework uses the LEGATO Gripper, a versatile wearable gripper, to maintain consistent physical interactions across different robots. 
        During data collection, our gripper records the trajectories and grasping actions of the wearable gripper, as well as visual observations from its egocentric stereo camera. 
        Policies trained from demonstrations by humans or teleoperated robots using the tool can be deployed across various robots equipped with the same gripper. 
        Motion retargeting enables these trajectories to be executed on different robots without requiring robot-specific training data.
      </p>
    </td>
  </tr>
</table>

<hr>
<h1 align="center">Wearable Gripper Design</h1>

<table border="0" cellspacing="10" cellpadding="0" align="center">
  <tbody>
    <tr> 
      <td align="center" valign="middle">
        <video muted autoplay loop width="394">
          <source src="./src/video/mechanism.mp4"  type="video/mp4">
        </video>
      </td>
    </tr> 
  </tbody> 
</table>
<table border="0" cellspacing="10" cellpadding="0" align="center">
  <tbody>
    <tr>
      <td align="center" valign="middle">
        <a href="./src/figure/hardware.png"> 
          <img src="./src/figure/hardware.png" style="width:394;">
        </a>
      </td>
      <td align="center" valign="middle">
        <video muted autoplay loop width="394">
          <source src="./src/video/assembly.mp4"  type="video/mp4">
        </video>
      </td>
    </tr> 
  </tbody> 
</table>

<table align=center width=800px>
  <tr>
    <td> 
      <p align="justify" width="20%">
        The LEGATO Gripper is designed for both collecting human demonstrations and robot deployment.
        It features a shared actuated gripper with adaptable handles, ensuring reliable human handling and consistent grasping across robots while minimizing parts.
      </p>
    </td>
  </tr>
</table>

<table border="0" cellspacing="10" cellpadding="0" align="center">
  <tbody>
    <tr>
      <td align="center" valign="middle">
        <video muted controls loop width="394">
          <source src="./src/video/usage_human.mp4"  type="video/mp4">
        </video>
      </td>
      <td align="center" valign="middle">
        <video muted controls loop width="394">
          <source src="./src/video/usage_robot.mp4"  type="video/mp4">
        </video>
      </td>
    </tr>
  </tbody>
</table>

<table align=center width=800px>
  <tr>
    <td> 
      <p align="justify" width="20%">
        A human demonstrator can directly perform tasks using the the LEGATO Gripper by carrying it. 
        The LEGATO Gripper is easily installable on various robots, held securely by their original grippers, and is ready for immediate use.
      </p>
    </td>
  </tr>
</table>

<hr>
<h1 align="center">Whole-body Motion Retargeting</h1>

<table border="0" cellspacing="10" cellpadding="0" align="center">
  <tbody>
    <tr>
      <td align="center" valign="middle">
        <video class="lazy-video" muted loop width="798">
          <source src="./src/video/whole-body_ik.mp4"  type="video/mp4">
        </video>
      </td>
    </tr> 
  </tbody> 
</table>

<table align=center width=800px>
  <tr>
    <td> 
      <p align="justify" width="20%">
        Motion retargeting through IK optimization adeptly navigates the kinematic differences and constraints across robot embodiments, exploiting kinematic redundancy without requiring additional robot-specific demonstrations for deployment.
      </p>
    </td>
  </tr>
</table>

<hr>
<h1 align="center">Real-Robot Deployment</h1>
<table align=center width=800px>
  <tr>
    <td> 
      <p align="justify" width="20%">
        We trained visuomotor policies on direct human demonstrations and successfully deployed them on the <i>Panda</i> robot system. Our method succeeded in 16 trials of the <i>Closing the lid</i> task, 13 trials of the <i>Cup shelving</i> task, and 14 trials of the <i>Ladle reorganization</i> task, respectively.
      </p>
    </td>
  </tr>
</table>

<table border="0" cellspacing="10" cellpadding="0" align="center">
  <tbody>
    <tr>
      <td align="center" valign="middle">
        <video class="lazy-video" muted loop  width="598">
          <source src="./src/video/real_lid.mp4"  type="video/mp4">
        </video>
      </td>
    </tr>
    <tr>
      <td align="center" valign="middle">
        <video class="lazy-video" muted loop  width="598">
          <source src="./src/video/real_cup.mp4"  type="video/mp4">
        </video>
      </td>
    </tr>
    <tr>
      <td align="center" valign="middle">
        <video class="lazy-video" muted loop  width="598">
          <source src="./src/video/real_ladle.mp4"  type="video/mp4">
        </video>
      </td>
    </tr>
  </tbody>
</table>

<hr>
<h1 align="center">Simulation Evaluation</h1>

<table border="0" cellspacing="10" cellpadding="0" align="center">
  <tbody>
    <tr>
      <td align="center" valign="middle">
        <a href="./src/figure/benchmark.png">
          <img src="./src/figure/benchmark.png" style="width:798;">
        </a>
      </td>
    </tr>
  </tbody>
</table>

<table align=center width=800px>
  <tr>
    <td>
      <p align="justify" width="20%">
    	On average, LEGATO  outperforms the other methods in cross-embodiment deployment by 28.9%, 10.5%, and 21.1%, compared to <a href="https://arxiv.org/abs/2108.03298">BC-RNN</a>, <a href="https://arxiv.org/abs/2303.04137v5">Diffusion Policy</a>, and the self-variant of LEGATO trained only on SE3 (LEGATO (SE3)), respectively.
    	Notably, unlike the baselines that only achieved high success rates on specific robot bodies, typically the <i>Abstract</i> embodiment used for training, LEGATO demonstrates consistent success across different embodiments.
      </p>
    </td>
  </tr>
</table>

<table border="0" cellspacing="10" cellpadding="0" align="center">
  <tbody>
    <tr>
      <td align="center" valign="middle">
        <video class="lazy-video" muted loop width="598">
          <source src="./src/video/sim_lid.mp4"  type="video/mp4">
        </video>
      </td>
    </tr>
    <tr>
      <td align="center" valign="middle">
        <video class="lazy-video" muted loop width="598">
          <source src="./src/video/sim_cup.mp4"  type="video/mp4">
        </video>
      </td>
    </tr>
    <tr>
      <td align="center" valign="middle">
        <video class="lazy-video" muted loop width="598">
          <source src="./src/video/sim_ladle.mp4"  type="video/mp4">
        </video>
      </td>
    </tr>
  </tbody>
</table>

<hr>
<center><h1>Citation</h1></center>

<table align=center width=800px>
  <tr>
    <td>
    <!-- <left> -->
    <pre><code style="display:block; overflow-x: auto">
      @misc{seo2024legato,
        title={LEGATO: Cross-Embodiment Visual Imitation Using a Grasping Tool},
        author={Seo, Mingyo and Park, H. Andy and Yuan, Shenli and Zhu, Yuke and
          and Sentis, Luis},
        year={2024}
        eprint={2411.03682},
        archivePrefix={arXiv},
        primaryClass={cs.RO}
      }
    </code></pre>
    <!-- </left> -->
    </td>
  </tr>
</table>

<div class="page-width-background">
<div style="height: 4px;"></div>
<center><h2 align="center">Acknowledgement</h2></center>
<table align=center width=800px>
  <tr>
    <td> 
      <p align="justify" width="20%">
        This work was conducted during Mingyo Seo's internship at the AI Institute. We thank Rutav Shah and Minkyung Kim for providing feedback on this manuscript. We thank Osman Dogan Yirmibesoglu for designing the fin ray style compliant fingers and helping with hardware prototyping. We thank Mitchell Pryor and Fabian Parra for their support with the real Spot demonstration. We acknowledge the support of the AI Institute and the Office of Naval Research (N00014-22-1-2204).
      </p>
    </td>
  </tr>
</table>
<div style="height: 16px;"></div>
</div>
