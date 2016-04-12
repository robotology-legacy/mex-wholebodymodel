classdef wbmColor
    properties(Constant)
        % Small color list to colorize the simulation of the robot with some nice colors:
        % The color list is based on the color map of R for Statistical Computing (https://www.r-project.org).
        % Source: <http://research.stowers-institute.org/efg/R/Color/Chart/ColorChart.pdf>
        %                                              idx:
        antiquewhite         = [250 235 215] ./ 255; % 3
        azure                = [240 255 255] ./ 255; % 13
        burlywood            = [222 184 135] ./ 255; % 37
        chocolate4           = [139 69   19] ./ 255; % 56
        darkgrey             = [169 169 169] ./ 255; % 82
        darkmagenta          = [139 0   139] ./ 255; % 84
        cornsilk             = [255 248 220] ./ 255; % 63
        darkblue             = [0   0   139] ./ 255; % 73
        darkgreen            = [0   100   0] ./ 255; % 81
        greenyellow          = [173 255  47] ./ 255; % 259
        lavender             = [230 230 250] ./ 255; % 387
        lightgoldenrod       = [238 221 130] ./ 255; % 410
        lightgoldenrodyellow = [250 250 210] ./ 255; % 415
        lightgray            = [211 211 211] ./ 255; % 416
        lightgreen           = [144 238 144] ./ 255; % 417
        lightsalmon          = [255 160 122] ./ 255; % 424
        lightskyblue         = [135 206 250] ./ 255; % 430
        lightsteelblue1      = [202 225 255] ./ 255; % 438
        lightyellow          = [255 255 224] ./ 255; % 443
        maroon4              = [139 28   98] ./ 255; % 459
        mediumorchid4        = [122 55  139] ./ 255; % 466
        mediumseagreen       = [60  179 113] ./ 255; % 472
        mediumslateblue      = [123 104 238] ./ 255; % 473
        mediumvioletred      = [199 21  133] ./ 255; % 476
        midnightblue         = [25  25  112] ./ 255; % 477
        mintcream            = [245 255 250] ./ 255; % 478
        mistyrose            = [255 228 225] ./ 255; % 479
        moccasin             = [255 228 181] ./ 255; % 484
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
        purple               = [160 32  240] ./ 255; % 547
        red4                 = [139 0     0] ./ 255; % 556
        royalblue4           = [39  64  139] ./ 255; % 566
        sandybrown           = [244 164  96] ./ 255; % 573
        seagreen             = [46  139  87] ./ 255; % 574
        seashell             = [255 245 238] ./ 255; % 579
        slategray2           = [185 211 238] ./ 255; % 601
        snow                 = [255 250 250] ./ 255; % 605
        steelblue            = [70  130 180] ./ 255; % 615
        wheat                = [245 222 179] ./ 255; % 646
        yellowgreen          = [154 205  50] ./ 255; % 657
    end
end
