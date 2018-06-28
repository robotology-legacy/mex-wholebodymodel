% Copyright (C) 2015-2018, by Martin Neururer
% Author: Martin Neururer
% E-mail: martin.neururer@student.tuwien.ac.at / martin.neururer@gmail.com
% Date:   January-May, 2018
%
% Departments:
%   Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia and
%   Automation and Control Institute - TU Wien.
%
% This file is part of the Whole-Body Model Library for Matlab (WBML).
%
% The development of the WBM-Library was made in the context of the master
% thesis "Learning Task Behaviors for Humanoid Robots" and is an extension
% for the Matlab MEX whole-body model interface, which was supported by the
% FP7 EU-project CoDyCo (No. 600716, ICT-2011.2.1 Cognitive Systems and
% Robotics (b)), <http://www.codyco.eu>.
%
% Permission is granted to copy, distribute, and/or modify the WBM-Library
% under the terms of the GNU Lesser General Public License, Version 2.1
% or any later version published by the Free Software Foundation.
%
% The WBM-Library is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU Lesser General Public License for more details.
%
% A copy of the GNU Lesser General Public License can be found along
% with the WBML. If not, see <http://www.gnu.org/licenses/>.

classdef wbmColor
    % :class:`!wbmColor` is a *utility class* with a list of constant member
    % variables of predefined RGB-triplets to colorize the simulation of the
    % robot with some nice colors.
    %
    % The color list is based on the color map of *R for Statistical Computing*
    % (`www.r-project.org <https://www.r-project.org>`_).
    %
    % Further details about the R colors are available in the technical note of
    % the *Stowers Institute for Medical Research*:
    % `<http://research.stowers.org/mcm/efg/R/Color/Chart/index.htm>`_.
    properties(Constant)
        % R colors:
        % Source: <http://research.stowers.org/mcm/efg/R/Color/Chart/ColorChart.pdf>

                                                     % idx:
        aliceblue            = [240 248 255] ./ 255; % 2
        antiquewhite         = [250 235 215] ./ 255; % 3
        azure                = [240 255 255] ./ 255; % 13
        burlywood            = [222 184 135] ./ 255; % 37
        chocolate4           = [139 69   19] ./ 255; % 56
        coral                = [255 127  80] ./ 255; % 57
        cornflowerblue       = [100 149 237] ./ 255; % 62
        cornsilk             = [255 248 220] ./ 255; % 63
        darkblue             = [0   0   139] ./ 255; % 73
        darkgray             = [169 169 169] ./ 255; % 80
        darkgreen            = [0   100   0] ./ 255; % 81
        darkmagenta          = [139 0   139] ./ 255; % 84
        deeppink             = [255 20  147] ./ 255; % 116
        deepskyblue          = [0   191 255] ./ 255; % 121
        dimgray              = [105 105 105] ./ 255; % 126
        dodgerblue           = [30  144 255] ./ 255; % 128
        dodgerblue4          = [16  78  139] ./ 255; % 132
        firebrick            = [178 34   34] ./ 255; % 133
        floralwhite          = [255 250 240] ./ 255; % 138
        forestgreen          = [34  139  34] ./ 255; % 139
        gold                 = [255 215   0] ./ 255; % 142
        gray                 = [190 190 190] ./ 255; % 152
        gray80               = [204 204 204] ./ 255; % 233
        greenyellow          = [173 255  47] ./ 255; % 259
        indianred1           = [255 106 106] ./ 255; % 373
        khaki                = [240 230 140] ./ 255; % 382
        lavender             = [230 230 250] ./ 255; % 387
        lightgoldenrod       = [238 221 130] ./ 255; % 410
        lightgoldenrodyellow = [250 250 210] ./ 255; % 415
        lightgray            = [211 211 211] ./ 255; % 416
        lightgreen           = [144 238 144] ./ 255; % 417
        lightsalmon          = [255 160 122] ./ 255; % 424
        lightskyblue         = [135 206 250] ./ 255; % 430
        lightsteelblue       = [176 196 222] ./ 255; % 438
        lightsteelblue1      = [202 225 255] ./ 255; % 439
        lightyellow          = [255 255 224] ./ 255; % 443
        maroon               = [176 48   96] ./ 255; % 455
        maroon4              = [139 28   98] ./ 255; % 459
        mediumorchid4        = [122 55  139] ./ 255; % 466
        mediumseagreen       = [60  179 113] ./ 255; % 472
        mediumslateblue      = [123 104 238] ./ 255; % 473
        mediumvioletred      = [199 21  133] ./ 255; % 476
        midnightblue         = [25  25  112] ./ 255; % 477
        mintcream            = [245 255 250] ./ 255; % 478
        mistyrose            = [255 228 225] ./ 255; % 479
        moccasin             = [255 228 181] ./ 255; % 484
        navajowhite          = [255 222 173] ./ 255; % 485
        navyblue             = [0   0   128] ./ 255; % 491
        olivedrab            = [107 142  35] ./ 255; % 493
        olivedrab1           = [192 255  62] ./ 255; % 494
        orange               = [255 165   0] ./ 255; % 498
        orangered            = [255 69    0] ./ 255; % 503
        orchid               = [218 112 214] ./ 255; % 508
        palegoldenrod        = [238 232 170] ./ 255; % 513
        palegreen            = [152 251 152] ./ 255; % 514
        paleturquoise        = [175 238 238] ./ 255; % 519
        papayawhip           = [255 239 213] ./ 255; % 529
        peachpuff            = [255 218 185] ./ 255; % 530
        plum                 = [221 160 221] ./ 255; % 541
        purple               = [160 32  240] ./ 255; % 547
        red4                 = [139 0     0] ./ 255; % 556
        royalblue            = [65  105 225] ./ 255; % 562
        royalblue4           = [39  64  139] ./ 255; % 566
        saddlebrown          = [139 69   19] ./ 255; % 567
        sandybrown           = [244 164  96] ./ 255; % 573
        seagreen             = [46  139  87] ./ 255; % 574
        seashell             = [255 245 238] ./ 255; % 579
        seashell4            = [139 134 130] ./ 255; % 583
        sienna               = [160  82  45] ./ 255; % 584
        skyblue              = [135 206 235] ./ 255; % 589
        slategray2           = [185 211 238] ./ 255; % 601
        snow                 = [255 250 250] ./ 255; % 605
        steelblue            = [70  130 180] ./ 255; % 615
        tan                  = [210 180 140] ./ 255; % 620
        tan1                 = [255 165  79] ./ 255; % 621
        thistle              = [216 191 216] ./ 255; % 625
        tomato               = [255 99   71] ./ 255; % 630
        turquoise            = [64  224 208] ./ 255; % 635
        turquoise1           = [0   245 255] ./ 255; % 636
        violetred            = [208 32  144] ./ 255; % 641
        violetred4           = [139 34   82] ./ 255; % 645
        wheat                = [245 222 179] ./ 255; % 646
        whitesmoke           = [245 245 245] ./ 255; % 651
        yellowgreen          = [154 205  50] ./ 255; % 657
    end
end
