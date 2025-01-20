/*
 @licstart  The following is the entire license notice for the JavaScript code in this file.

 The MIT License (MIT)

 Copyright (C) 1997-2020 by Dimitri van Heesch

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the "Software"), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge, publish, distribute,
 sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or
 substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 @licend  The above is the entire license notice for the JavaScript code in this file
*/
var NAVTREE =
[
  [ "ReveilLib", "index.html", [
    [ "sciplot's API Reference", "index.html", null ],
    [ "Setting up the ReveilLib depot", "md_docs_2tutorials_2depot-setup.html", null ],
    [ "Reckless Controller Usage Guide", "md_docs_2tutorials_2reckless.html", [
      [ "Getting Started", "md_docs_2tutorials_2reckless.html#autotoc_md4", [
        [ "Includes", "md_docs_2tutorials_2reckless.html#autotoc_md5", null ],
        [ "Chassis", "md_docs_2tutorials_2reckless.html#autotoc_md6", null ],
        [ "Odometry", "md_docs_2tutorials_2reckless.html#autotoc_md7", null ],
        [ "Reckless controller", "md_docs_2tutorials_2reckless.html#autotoc_md8", null ]
      ] ],
      [ "Using the Reckless controller", "md_docs_2tutorials_2reckless.html#autotoc_md9", [
        [ "Motion", "md_docs_2tutorials_2reckless.html#autotoc_md10", null ],
        [ "Correction", "md_docs_2tutorials_2reckless.html#autotoc_md11", null ],
        [ "Stopping", "md_docs_2tutorials_2reckless.html#autotoc_md12", null ],
        [ "Target point", "md_docs_2tutorials_2reckless.html#autotoc_md13", null ],
        [ "Early Drop", "md_docs_2tutorials_2reckless.html#autotoc_md14", null ]
      ] ]
    ] ],
    [ "About", "md_sciplot_2docs_2website_2about.html", [
      [ "License", "md_sciplot_2docs_2website_2about.html#autotoc_md16", null ]
    ] ],
    [ "API", "md_sciplot_2docs_2website_2api.html", null ],
    [ "FAQ", "md_sciplot_2docs_2website_2faq.html", [
      [ "How are my plots generated?", "md_sciplot_2docs_2website_2faq.html#autotoc_md19", null ],
      [ "Something is not working for me. Is it a known issue?", "md_sciplot_2docs_2website_2faq.html#autotoc_md20", null ],
      [ "How do I report issues?", "md_sciplot_2docs_2website_2faq.html#autotoc_md21", null ],
      [ "How can I contribute?", "md_sciplot_2docs_2website_2faq.html#autotoc_md22", null ],
      [ "How can I cite?", "md_sciplot_2docs_2website_2faq.html#autotoc_md23", null ]
    ] ],
    [ "<img src=\"img/logo.svg\" alt=\"sciplot\"/>", "md_sciplot_2docs_2website_2index.html", [
      [ "! WARNING ! sciplot has been redesigned and there were breaking API changes between v0....", "md_sciplot_2docs_2website_2index.html#autotoc_md25", null ]
    ] ],
    [ "Installation", "md_sciplot_2docs_2website_2installation.html", [
      [ "Download", "md_sciplot_2docs_2website_2installation.html#autotoc_md27", null ],
      [ "Installation by copying", "md_sciplot_2docs_2website_2installation.html#autotoc_md28", null ],
      [ "Installation using FetchContent", "md_sciplot_2docs_2website_2installation.html#autotoc_md29", null ],
      [ "Installation using CMake", "md_sciplot_2docs_2website_2installation.html#autotoc_md30", null ],
      [ "Installation failed. What do I do?", "md_sciplot_2docs_2website_2installation.html#autotoc_md31", null ]
    ] ],
    [ "Known issues", "md_sciplot_2docs_2website_2known__issues.html", [
      [ "I get compiler errors when compiling for Windows", "md_sciplot_2docs_2website_2known__issues.html#autotoc_md33", null ],
      [ "I still have problems when compiling for Windows", "md_sciplot_2docs_2website_2known__issues.html#autotoc_md34", null ],
      [ "I have problems setting a grid for my plot", "md_sciplot_2docs_2website_2known__issues.html#autotoc_md35", null ],
      [ "I have a different issue", "md_sciplot_2docs_2website_2known__issues.html#autotoc_md36", null ]
    ] ],
    [ "Tutorials", "md_sciplot_2docs_2website_2tutorials.html", [
      [ "Plotting sine functions", "md_sciplot_2docs_2website_2tutorials.html#autotoc_md38", null ],
      [ "Plotting Bessel functions", "md_sciplot_2docs_2website_2tutorials.html#autotoc_md39", null ],
      [ "Plotting filled curves", "md_sciplot_2docs_2website_2tutorials.html#autotoc_md40", null ],
      [ "Using logarithmic axes", "md_sciplot_2docs_2website_2tutorials.html#autotoc_md41", null ],
      [ "Plotting multiple plots", "md_sciplot_2docs_2website_2tutorials.html#autotoc_md42", null ],
      [ "Ploting trigonometric functions", "md_sciplot_2docs_2website_2tutorials.html#autotoc_md43", null ],
      [ "Plotting boxes", "md_sciplot_2docs_2website_2tutorials.html#autotoc_md44", null ],
      [ "Plotting boxes with custom tick labels", "md_sciplot_2docs_2website_2tutorials.html#autotoc_md45", null ],
      [ "Plotting broken curves when NaN values are present", "md_sciplot_2docs_2website_2tutorials.html#autotoc_md46", null ],
      [ "Plotting a 3D helix plot", "md_sciplot_2docs_2website_2tutorials.html#autotoc_md47", null ],
      [ "Plotting multiple mixed 2D and 3D plots", "md_sciplot_2docs_2website_2tutorials.html#autotoc_md48", null ],
      [ "Changing plots in a figure", "md_sciplot_2docs_2website_2tutorials.html#autotoc_md49", null ],
      [ "Integration with CMake-based projects", "md_sciplot_2docs_2website_2tutorials.html#autotoc_md50", null ]
    ] ],
    [ "Code styling & standards for ReveilLib", "md_Style.html", [
      [ "Capitalization", "md_Style.html#autotoc_md60", null ]
    ] ],
    [ "Classes", "annotated.html", [
      [ "Class List", "annotated.html", "annotated_dup" ],
      [ "Class Index", "classes.html", null ],
      [ "Class Hierarchy", "hierarchy.html", "hierarchy" ],
      [ "Class Members", "functions.html", [
        [ "All", "functions.html", "functions_dup" ],
        [ "Functions", "functions_func.html", "functions_func" ],
        [ "Variables", "functions_vars.html", null ]
      ] ]
    ] ],
    [ "Files", "files.html", [
      [ "File List", "files.html", "files_dup" ],
      [ "File Members", "globals.html", [
        [ "All", "globals.html", null ],
        [ "Functions", "globals_func.html", null ],
        [ "Macros", "globals_defs.html", null ]
      ] ]
    ] ]
  ] ]
];

var NAVTREEINDEX =
[
"AxisLabelSpecs_8hpp_source.html",
"classrev_1_1PilonsSegment.html#aa6b3b674d53d1016ade8b4c0b3abfabf",
"classsciplot_1_1LineSpecs.html",
"classsciplot_1_1TitleSpecsOf.html#a6184a57105fcd3e4dd37e89692b1b7bb",
"structCatch_1_1StringMaker.html"
];

var SYNCONMSG = 'click to disable panel synchronization';
var SYNCOFFMSG = 'click to enable panel synchronization';