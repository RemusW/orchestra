defmodule InteropProxy.Mixfile do
  use Mix.Project

  def project do
    [
      app: :interop_proxy,
      version: "0.1.0",
      elixir: "~> 1.5",
      start_permanent: Mix.env == :prod,
      deps: deps()
    ]
  end

  def application do
    [
      extra_applications: [:logger, :httpoison],
      mod: {InteropProxy.Application, []}
    ]
  end

  defp deps do
    [
      {:httpoison, "~> 0.13.0"},
      {:poison, "~> 3.1.0"}
    ]
  end
end
