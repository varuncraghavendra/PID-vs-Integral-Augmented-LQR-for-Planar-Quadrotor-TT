function run_benchmarks(caseID, useResearch)
% run_benchmarks.m
%
% Run pre-defined benchmark cases:
%   caseID:
%     1 -> Circle, slow speed (baseline or research)
%     2 -> Circle, high speed
%     3 -> Square, medium speed
%
%   useResearch:
%     true  -> use quad_pid_lqr_research
%     false -> use quad_pid_lqr (baseline)
%
% Examples:
%   run_benchmarks(1);          % case 1, research (default)
%   run_benchmarks(2,false);    % case 2, baseline gains

    if nargin < 1 || isempty(caseID)
        % If no case given, run all 3 with research gains
        fprintf('No caseID given; running all 3 with research gains.\n');
        for cid = 1:3
            run_benchmarks(cid, true);
        end
        return;
    end

    if nargin < 2 || isempty(useResearch)
        useResearch = true;
    end

    if useResearch
        runner = @quad_pid_lqr_research;
        variantLabel = 'Research';
    else
        runner = @quad_pid_lqr;
        variantLabel = 'Baseline';
    end

    switch caseID
        case 1
            trajType  = 'circle';
            R         = 3;
            v_ref     = 1.0;   % slow
            numLaps   = 3;
            caseLabel = sprintf('Case 1: Circle slow (%s)', variantLabel);

        case 2
            trajType  = 'circle';
            R         = 3;
            v_ref     = 3.0;   % fast
            numLaps   = 3;
            caseLabel = sprintf('Case 2: Circle fast (%s)', variantLabel);

        case 3
            trajType  = 'square';
            R         = 3;
            v_ref     = 1.8;
            numLaps   = 3;
            caseLabel = sprintf('Case 3: Square (%s)', variantLabel);

        otherwise
            error('Unknown caseID. Use 1, 2, or 3.');
    end

    fprintf('\n\n==================== %s ====================\n', caseLabel);
    runner(trajType, R, v_ref, numLaps, caseLabel);
end
