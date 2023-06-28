package logging

import (
	prettyconsole "github.com/thessem/zap-prettyconsole"
	"go.uber.org/zap"
)

func Logger() *zap.SugaredLogger {
	var logger, _ = prettyconsole.NewConfig().Build()
	return logger.Sugar()
}
