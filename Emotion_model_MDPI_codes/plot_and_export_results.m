function plot_and_export_results(tFree, xFree, tLqr, xLqr, uLqr, p, outputFolder)
%PLOT_AND_EXPORT_RESULTS Creates publication-quality scenario figures.

figComparison = figure('Color','w','Position',[100,100,1100,700]);
tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
for i = 1:6
    nexttile;
    plot(tFree,xFree(:,i),'k--','LineWidth',1.5); hold on;
    plot(tLqr,xLqr(:,i),'b-','LineWidth',1.8);
    add_event_markers(p); grid on;
    xlabel('Time (min)'); ylabel('\Delta emotion'); title(p.emotionNames{i});
    legend('Uncontrolled','Saturated LQR','Location','best');
end
export_figure(figComparison, outputFolder, [p.scenarioTag '_open_loop_lqr_comparison']);

figOpen = figure('Color','w','Position',[100,100,1100,700]);
tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
for i = 1:6
    nexttile;
    plot(tFree,xFree(:,i),'k-','LineWidth',1.8); hold on;
    add_event_markers(p); yline(0,':','Color',[.45,.45,.45]); grid on;
    xlabel('Time (min)'); ylabel('\Delta emotion'); title(p.emotionNames{i});
end
export_figure(figOpen, outputFolder, [p.scenarioTag '_open_loop_emotions']);

figRobot = figure('Color','w','Position',[100,100,1000,650]);
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
for j = 1:4
    nexttile;
    plot(tLqr,uLqr(:,j),'r-','LineWidth',1.8); hold on;
    yline(p.u0(j),'k--','Nominal'); grid on; ylim([-.05,1.05]);
    xlabel('Time (min)'); ylabel('Command level'); title(p.inputNames{j});
end
export_figure(figRobot, outputFolder, [p.scenarioTag '_lqr_robot_commands']);
end

function add_event_markers(p)
for k = 1:numel(p.eventEndTimes)
    xline(p.eventEndTimes(k),':',sprintf('Event %d ends',k), ...
        'LabelVerticalAlignment','bottom');
end
end

function export_figure(fig, outputFolder, baseName)
print(fig, fullfile(outputFolder,[baseName '.png']), '-dpng','-r300');
% print(fig, fullfile(outputFolder,[baseName '.pdf']), '-dpdf','-r300');
end
