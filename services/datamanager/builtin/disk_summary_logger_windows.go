package builtin

func (poller *diskSummaryLogger) logDiskUsage(dir string) {
	poller.logger.Debug("can't log disk usage yet on windows")
}
